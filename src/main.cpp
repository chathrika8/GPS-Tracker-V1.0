/*
 * GPS Tracker — Main Entry Point
 * ESP32-C3 + SIM800L + NEO-6M + SH1107 OLED
 *
 * FreeRTOS tasks:
 *   - GPS Task:      5 Hz NMEA parsing
 *   - Buffer Task:   10s packet writes to SPIFFS (fire-and-forget, no retry)
 *   - Uplink Task:   Continuous GPRS drain
 *   - Display Task:  5 Hz UI rendering
 *   - Button Task:   50ms polling
 *   - Schedule Task: 1 min tracking window checks
 */

#include <Arduino.h>
#include <Wire.h>
#include <nvs_flash.h>
#include "config.h"
#include "gps_manager.h"
#include "gsm_manager.h"
#include "wifi_manager.h"
#include "packet_buffer.h"
#include "display_manager.h"
#include "server_comm.h"
#include "command_handler.h"
#include "schedule_manager.h"
#include "ota_manager.h"
#include "button_handler.h"
#include "power_manager.h"

// ── Shared device state (mutex-protected) ──
DeviceState deviceState;
SemaphoreHandle_t stateMutex;

// ── Task handles ──
TaskHandle_t gpsTaskHandle      = NULL;
TaskHandle_t bufferTaskHandle   = NULL;
TaskHandle_t uplinkTaskHandle   = NULL;
TaskHandle_t displayTaskHandle  = NULL;
TaskHandle_t buttonTaskHandle   = NULL;
TaskHandle_t scheduleTaskHandle = NULL;

// ── Forward declarations ──
void gpsTask(void* param);
void bufferTask(void* param);
void uplinkTask(void* param);
void displayTask(void* param);
void buttonTask(void* param);
void scheduleTask(void* param);

void setup() {
    Serial.begin(115200);
    Serial.println("\n[GPS-TRACKER] Booting " FW_VERSION "...");

    // Initialize I2C and Display FIRST so we don't have a black screen
    Wire.begin(I2C_SDA, I2C_SCL);
    displayManager.begin();
    
    // Initialize NVS (required for Preferences/Schedule persist)
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Create mutex for shared state
    stateMutex = xSemaphoreCreateMutex();
    memset(&deviceState, 0, sizeof(DeviceState));
    strncpy(deviceState.fw_version, FW_VERSION, sizeof(deviceState.fw_version));

    // Initialize modules
    gpsManager.begin();
    wifiManager.begin();
    packetBuffer.begin();
    packetBuffer.clear();  // Discard any packets left from previous session — avoids stale-data reupload on boot
    buttonHandler.begin();
    scheduleManager.begin();
    powerManager.begin();
    otaManager.begin();

    // GSM init shifted to uplinkTask to prevent blocking the UI
    Serial.println("[GPS-TRACKER] Modules initialized. Starting tasks...");

    // ── Create FreeRTOS tasks ──
    xTaskCreate(gpsTask,      "GPS",      4096, NULL, 3, &gpsTaskHandle);
    xTaskCreate(bufferTask,   "Buffer",   4096, NULL, 2, &bufferTaskHandle);
    xTaskCreate(uplinkTask,   "Uplink",   16384, NULL, 2, &uplinkTaskHandle);
    xTaskCreate(displayTask,  "Display",  4096, NULL, 1, &displayTaskHandle);
    xTaskCreate(buttonTask,   "Button",   2048, NULL, 1, &buttonTaskHandle);
    xTaskCreate(scheduleTask, "Schedule", 2048, NULL, 1, &scheduleTaskHandle);

    Serial.println("[GPS-TRACKER] All tasks started.");
}

void loop() {
    // FreeRTOS tasks handle everything; main loop is idle
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// ─────────────────────────────────────────────
// GPS Task — 5 Hz NMEA parsing
// ─────────────────────────────────────────────
void gpsTask(void* param) {
    TickType_t lastWake = xTaskGetTickCount();
    for (;;) {
        gpsManager.update();

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            gpsManager.fillState(deviceState);
            xSemaphoreGive(stateMutex);
        }

        // Run at 5 Hz (200 ms)
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(200));
    }
}

// ─────────────────────────────────────────────
// Buffer Task — Save GPS packets to SPIFFS
// ─────────────────────────────────────────────
void bufferTask(void* param) {
    // Persist the most recent fix to NVS once a minute so the next boot can
    // seed UBX-AID-INI with a coarse position. Cheap insurance against the
    // NEO-6M's lack of a backup battery on most cheap dev boards.
    static uint32_t lastNvsWrite = 0;

    for (;;) {
        bool moving = false;

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            double spd = deviceState.speed_kmh;
            // Use SPEED_THRESHOLD for selecting buffer interval (fast vs slow).
            moving = spd > SPEED_THRESHOLD_KMH;

            if (deviceState.gps_fix) {
                GPSPacket pkt;
                pkt.timestamp  = deviceState.utc_epoch;
                pkt.latitude   = deviceState.latitude;
                pkt.longitude  = deviceState.longitude;
                pkt.altitude_m = deviceState.altitude_m;
                // Clamp to 0 below noise floor — matches display behaviour
                pkt.speed_kmh  = (deviceState.speed_kmh < 1.5f) ? 0.0f : deviceState.speed_kmh;
                pkt.course     = deviceState.course;
                pkt.satellites = deviceState.satellites;
                pkt.hdop       = deviceState.hdop;
                pkt.gsm_signal = deviceState.signal_percent;
                pkt.battery_v  = deviceState.battery_voltage;

                packetBuffer.store(pkt);

                if (millis() - lastNvsWrite > 60000UL) {
                    gpsManager.saveLastPositionToNVS(
                        deviceState.latitude,
                        deviceState.longitude,
                        (float)deviceState.altitude_m,
                        deviceState.utc_epoch);
                    lastNvsWrite = millis();
                }
            }
            xSemaphoreGive(stateMutex);
        }

        uint32_t interval = moving ? BUFFER_INTERVAL_MOVING : BUFFER_INTERVAL_IDLE;
        vTaskDelay(pdMS_TO_TICKS(interval));
    }
}

// ─────────────────────────────────────────────
// Uplink Task — Drain buffer via GPRS
// ─────────────────────────────────────────────
void uplinkTask(void* param) {
    Serial.println("[UPLINK] Initializing GSM asynchronously...");
    gsmManager.begin();

    if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
        deviceState.gsm_initialized = true;
        xSemaphoreGive(stateMutex);
    }

    bool agpsDone   = false;   // AssistNow / AID-INI runs once per boot
    uint32_t lastSendMs = 0;

    for (;;) {
        bool connected = gsmManager.isGprsConnected();
        bool registered = gsmManager.isRegistered();

        // ── AGPS bring-up: run once when GPRS first comes up ──
        if (connected && !agpsDone) {
            // Seed receiver with last-known position from NVS
            double lastLat = 0, lastLon = 0;
            float  lastAlt = 0;
            uint32_t lastTs = 0;
            if (gpsManager.loadLastPositionFromNVS(&lastLat, &lastLon, &lastAlt, &lastTs)) {
                Serial.printf("[AGPS] Seeding AID-INI with %.5f,%.5f\n", lastLat, lastLon);
                uint32_t utc = gsmManager.getNetworkUtcEpoch();
                gpsManager.injectAidIni(utc, lastLat, lastLon, lastAlt);
            }

#if ASSISTNOW_ENABLE
            // Pull AssistNow Online via the Worker; stream straight to GPS UART.
            static uint8_t agpsBuf[ASSISTNOW_MAX_LEN];
            size_t agpsLen = 0;
            if (serverComm.fetchAssistNow(agpsBuf, sizeof(agpsBuf), &agpsLen) && agpsLen > 0) {
                Serial.printf("[AGPS] Injecting %u bytes from AssistNow\n",
                              (unsigned)agpsLen);
                gpsManager.injectAssistNowBlob(agpsBuf, agpsLen);
                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    deviceState.agps_injected = true;
                    deviceState.agps_bytes    = (uint16_t)agpsLen;
                    xSemaphoreGive(stateMutex);
                }
            } else {
                Serial.println("[AGPS] AssistNow fetch failed — relying on AID-INI seed only");
            }
#endif
            agpsDone = true;
        }
        
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            deviceState.gprs_connected = connected;
            deviceState.registered_2g = registered;
            
            if (connected) {
                deviceState.signal_percent = gsmManager.getSignalPercent();
                strncpy(deviceState.network_name, gsmManager.getOperator().c_str(), sizeof(deviceState.network_name));
            }
            xSemaphoreGive(stateMutex);
        }

        // Periodically update battery voltage (every 60s)
        // Moved OUTSIDE the mutex to prevent blocking GPS/Display tasks (CBC takes ~1s)
        static uint32_t lastBattCheck = 0;
        if (millis() - lastBattCheck > 60000 || lastBattCheck == 0) {
            int pct = -1;
            float v = powerManager.readBatteryVoltage(&pct);
            if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                deviceState.battery_voltage = v;
                if (pct >= 0) deviceState.battery_percent = pct;
                xSemaphoreGive(stateMutex);
            }
            lastBattCheck = millis();
        }

        if (connected) {
            // ── Adaptive batching ──
            // Send when EITHER the buffer has a full batch, OR enough time
            // has elapsed since the last send. This keeps motion latency low
            // while still amortising TCP cost when stationary.
            bool isMoving = false;
            if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                isMoving = deviceState.speed_kmh > SPEED_THRESHOLD_KMH;
                xSemaphoreGive(stateMutex);
            }
            int       batchSize  = isMoving ? UPLINK_BATCH_SIZE_MOVING : UPLINK_BATCH_SIZE_IDLE;
            uint32_t  intervalMs = isMoving ? UPLINK_INTERVAL_MOVING   : UPLINK_INTERVAL_IDLE;
            uint32_t  bufCount   = packetBuffer.count();
            bool      timeReady  = (millis() - lastSendMs) >= intervalMs;
            bool      batchReady = bufCount >= (uint32_t)batchSize;

            if (bufCount > 0 && (batchReady || timeReady)) {
                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    deviceState.is_uploading = true;
                    xSemaphoreGive(stateMutex);
                }

                int sendCap = (int)bufCount;
                if (sendCap > batchSize) sendCap = batchSize;
                int sent = serverComm.sendBatch(sendCap);
                lastSendMs = millis();

                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    deviceState.is_uploading = false;
                    deviceState.last_http_code = serverComm.getLastHttpCode();
                    strncpy(deviceState.last_response, serverComm.getLastResponse().c_str(), sizeof(deviceState.last_response));
                    strncpy(deviceState.tcp_stage, serverComm.getTcpStage().c_str(), sizeof(deviceState.tcp_stage));
                    deviceState.tcp_hdr_sent = (uint16_t)serverComm.getTcpHdrSent();
                    deviceState.tcp_bod_sent = (uint16_t)serverComm.getTcpBodSent();
                    deviceState.tcp_bod_len  = (uint16_t)serverComm.getTcpBodLen();
                    if (sent > 0) {
                        deviceState.total_packets_sent += sent;
                        deviceState.last_uplink_time = millis();
                    }
                    xSemaphoreGive(stateMutex);
                }

                if (sent > 0) {
                    // After successful send, poll for commands
                    commandHandler.pollAndExecute();
                }
            }
        } else {
            gsmManager.ensureConnection();
        }

        // Check for manual ping test trigger
        bool triggerPing = false;
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            triggerPing = deviceState.trigger_ping_test;
            deviceState.trigger_ping_test = false;
            xSemaphoreGive(stateMutex);
        }
        
        if (triggerPing) {
            serverComm.testConnectivity();
            if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                deviceState.last_http_code = serverComm.getLastHttpCode();
                strncpy(deviceState.last_response, serverComm.getLastResponse().c_str(), sizeof(deviceState.last_response));
                xSemaphoreGive(stateMutex);
            }
        }

        // Yield briefly so other tasks aren't starved while we spin between
        // sends. Pacing is now driven by `batchReady || timeReady` above.
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ─────────────────────────────────────────────
// Display Task — 5 Hz UI rendering
// ─────────────────────────────────────────────
void displayTask(void* param) {
    TickType_t lastWake = xTaskGetTickCount();
    DeviceState localState;

    for (;;) {
        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
            wifiManager.fillState(deviceState);
            localState = deviceState; // 1 microsecond struct copy
            xSemaphoreGive(stateMutex);
        }
        
        // Render asynchronously via I2C without blocking the system Mutex lock
        displayManager.render(localState);

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(DISPLAY_REFRESH));
    }
}

// ─────────────────────────────────────────────
// Button Task — 50 ms polling
// ─────────────────────────────────────────────
void buttonTask(void* param) {
    for (;;) {
        ButtonEvent evt = buttonHandler.poll();

        if (evt != BTN_NONE) {
            if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
                switch (evt) {
                    case BTN_A_SHORT:
                        displayManager.nextScreen();
                        break;
                    case BTN_A_LONG:
                        displayManager.toggleDisplay();
                        break;
                    case BTN_B_SHORT:
                        // Trigger Connectivity Test (Ping)
                        if (displayManager.getCurrentScreen() == 4) {
                             deviceState.trigger_ping_test = true;
                        } else if (displayManager.getCurrentScreen() == 6) {
                             if (deviceState.wifi_enabled) {
                                 wifiManager.disable();
                             } else {
                                 wifiManager.enable();
                             }
                        }
                        break;
                    case BTN_B_LONG:
                        // Enter deep sleep
                        powerManager.enterDeepSleep(0); // 0 = no timed wake
                        break;
                    default:
                        break;
                }
                xSemaphoreGive(stateMutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE));
    }
}

// ─────────────────────────────────────────────
// Schedule Task — 1 min tracking window checks
// ─────────────────────────────────────────────
void scheduleTask(void* param) {
    for (;;) {
        scheduleManager.checkWindow(deviceState);
        vTaskDelay(pdMS_TO_TICKS(60000)); // Check every minute
    }
}
