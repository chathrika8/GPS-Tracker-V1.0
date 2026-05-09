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
#include "log.h"
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
#include "sms_manager.h"
#include "schedule_manager.h"
#include "ota_manager.h"
#include "button_handler.h"
#include "power_manager.h"

// ── Fallback defaults for v1.1 macros ──
// Override in include/config.h to tune; these just keep the build green
// against older config.h files that pre-date the AGPS additions.
#ifndef GPS_NVS_SAVE_INTERVAL_MS
#define GPS_NVS_SAVE_INTERVAL_MS   30000
#endif
#ifndef CELL_NVS_SAVE_INTERVAL_MS
#define CELL_NVS_SAVE_INTERVAL_MS  300000
#endif
#ifndef EPH_POLL_INTERVAL_MS
#define EPH_POLL_INTERVAL_MS       1800000
#endif
#ifndef EPH_MAX_AGE_SEC
#define EPH_MAX_AGE_SEC            10800
#endif
#ifndef AGPS_RESEED_KM
#define AGPS_RESEED_KM             50
#endif

// Approximate great-circle distance in km using the equirectangular
// projection. Plenty accurate for the "did the device move?" check.
static double approxKm(double lat1, double lon1, double lat2, double lon2) {
    const double DEG = 0.017453292519943295;        // π / 180
    double avg = ((lat1 + lat2) * 0.5) * DEG;
    double dx  = (lon2 - lon1) * DEG * cos(avg);
    double dy  = (lat2 - lat1) * DEG;
    return 6371.0 * sqrt(dx * dx + dy * dy);
}

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
    // Deliberately no Serial.begin(115200) — UART0's default pins (GPIO 20/21)
    // are wired to the SIM800L on this board, and any byte sent there before
    // gpsManager remaps Serial to GPS_RX/GPS_TX would jam the modem's AT line.
    LOG("\n[GPS-TRACKER] Booting " FW_VERSION "...");

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
    LOG("[GPS-TRACKER] Modules initialized. Starting tasks...");

    // ── Create FreeRTOS tasks ──
    xTaskCreate(gpsTask,      "GPS",      4096, NULL, 3, &gpsTaskHandle);
    xTaskCreate(bufferTask,   "Buffer",   4096, NULL, 2, &bufferTaskHandle);
    xTaskCreate(uplinkTask,   "Uplink",   24576, NULL, 2, &uplinkTaskHandle);
    xTaskCreate(displayTask,  "Display",  8192, NULL, 1, &displayTaskHandle);
    xTaskCreate(buttonTask,   "Button",   2048, NULL, 1, &buttonTaskHandle);
    xTaskCreate(scheduleTask, "Schedule", 2048, NULL, 1, &scheduleTaskHandle);

    LOG("[GPS-TRACKER] All tasks started.");
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
// Buffer Task — Save GPS packets to LittleFS + maintain AGPS NVS cache
// ─────────────────────────────────────────────
void bufferTask(void* param) {
    // GPS fix → NVS at GPS_NVS_SAVE_INTERVAL_MS so the next boot has a
    // recent coarse seed. Independent of the LittleFS upload buffer cadence.
    static uint32_t lastGpsNvsWrite = 0;

    for (;;) {
        bool moving = false;

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            double spd = deviceState.speed_kmh;
            moving = spd > SPEED_THRESHOLD_KMH;

            if (deviceState.gps_fix) {
                GPSPacket pkt;
                pkt.timestamp  = deviceState.utc_epoch;
                pkt.latitude   = deviceState.latitude;
                pkt.longitude  = deviceState.longitude;
                pkt.altitude_m = deviceState.altitude_m;
                pkt.speed_kmh  = (deviceState.speed_kmh < 1.5f) ? 0.0f : deviceState.speed_kmh;
                pkt.course     = deviceState.course;
                pkt.satellites = deviceState.satellites;
                pkt.hdop       = deviceState.hdop;
                pkt.gsm_signal = deviceState.signal_percent;
                pkt.battery_v  = deviceState.battery_voltage;
                packetBuffer.store(pkt);

                if (millis() - lastGpsNvsWrite > GPS_NVS_SAVE_INTERVAL_MS) {
                    gpsManager.saveLastPositionToNVS(
                        deviceState.latitude,
                        deviceState.longitude,
                        (float)deviceState.altitude_m,
                        deviceState.utc_epoch);
                    lastGpsNvsWrite = millis();
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
    LOG("[UPLINK] Initializing GSM asynchronously...");
    gsmManager.begin();

    if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
        deviceState.gsm_initialized = true;
        xSemaphoreGive(stateMutex);
    }

    // ── Phase 1: Boot-time AGPS — runs IMMEDIATELY using NVS only.
    //    Don't wait for GSM. The faster the receiver gets a coarse seed, the
    //    sooner it acquires its first fix. NVS has whatever was saved before
    //    last shutdown (could be sub-km if the device hasn't moved since).
    {
        double nvsLat = 0, nvsLon = 0, cellLat = 0, cellLon = 0;
        float  nvsAlt = 0;
        uint32_t nvsTs = 0, cellTs = 0;
        bool haveNvs  = gpsManager.loadLastPositionFromNVS(&nvsLat, &nvsLon, &nvsAlt, &nvsTs);
        bool haveCell = gpsManager.loadCellPositionFromNVS(&cellLat, &cellLon, &cellTs);

        // If both saved, prefer the more recent — same-trip GPS is more
        // accurate, but if cell was saved later (last thing before shutdown
        // in poor sky) it's the better hint.
        const char* src = "NONE ";
        double seedLat = 0, seedLon = 0;
        float  seedAlt = 0;
        if (haveNvs && (!haveCell || nvsTs >= cellTs)) {
            seedLat = nvsLat; seedLon = nvsLon; seedAlt = nvsAlt; src = "NVS  ";
        } else if (haveCell) {
            seedLat = cellLat; seedLon = cellLon; seedAlt = 0; src = "CELL ";
        }

        if (src[0] != 'N') {
            gpsManager.injectAidIni(0, seedLat, seedLon, seedAlt);
        }

        int replayed = gpsManager.replayEphemeridesFromNVS(0, EPH_MAX_AGE_SEC);

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            strncpy(deviceState.agps_source, src, sizeof(deviceState.agps_source));
            deviceState.agps_eph_count = (uint8_t)replayed;
            deviceState.agps_reseeded  = false;
            xSemaphoreGive(stateMutex);
        }
    }

    bool     agpsReseedTried = false;
    uint32_t lastSmsPoll = 0;  // seeded by smsManager.begin(); prevents immediate poll
    uint32_t lastCellSave    = 0;
    uint32_t lastSendMs      = 0;

    for (;;) {
        bool connected  = gsmManager.isGprsConnected();
        bool registered = gsmManager.isRegistered();

        // ── Phase 2: GSM-up re-seed.
        //    Once GPRS is up, fetch a fresh cell-tower position. If the GPS
        //    still hasn't fixed AND the cell location disagrees with what we
        //    already seeded by more than AGPS_RESEED_KM, the device must have
        //    moved while powered off. Push a fresh AID-INI so the receiver
        //    stops searching for satellites that aren't visible.
        if (connected && !agpsReseedTried) {
            double cLat, cLon;
            if (gsmManager.getCellLocation(&cLat, &cLon)) {
                uint32_t utc = gsmManager.getNetworkUtcEpoch();
                gpsManager.saveCellPositionToNVS(cLat, cLon, utc ? utc : (millis() / 1000));
                lastCellSave = millis();

                bool fixYet = false;
                double curLat = 0, curLon = 0;
                float  curAlt = 0;
                uint32_t curTs = 0;
                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    fixYet = deviceState.gps_fix;
                    xSemaphoreGive(stateMutex);
                }
                bool haveSeed = gpsManager.loadLastPositionFromNVS(&curLat, &curLon, &curAlt, &curTs);
                bool moved = haveSeed && approxKm(curLat, curLon, cLat, cLon) > AGPS_RESEED_KM;

                if (!fixYet && (moved || !haveSeed)) {
                    // Device was carried elsewhere, or has no NVS history at all.
                    gpsManager.injectAidIni(utc, cLat, cLon, 0.0f);
                    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                        strncpy(deviceState.agps_source, "RESEED", sizeof(deviceState.agps_source));
                        deviceState.agps_reseeded = true;
                        xSemaphoreGive(stateMutex);
                    }
                }
            }
            agpsReseedTried = true;
            
            // Init SMS config once GSM is up; seed poll timer so we wait
            // a full 30 s before the first poll (modem needs to settle)
            smsManager.begin();
            lastSmsPoll = millis();
        }

        // ── Phase 3: periodic cell-tower freshness anchor.
        //    Saves the latest cell fix to NVS so the next cold boot has a
        //    second source of truth alongside the GPS NVS save.
        if (connected && (millis() - lastCellSave) > CELL_NVS_SAVE_INTERVAL_MS) {
            double cLat, cLon;
            if (gsmManager.getCellLocation(&cLat, &cLon)) {
                uint32_t utc = gsmManager.getNetworkUtcEpoch();
                gpsManager.saveCellPositionToNVS(cLat, cLon, utc ? utc : (millis() / 1000));
            }
            lastCellSave = millis();
        }
        
        // Query OUTSIDE mutex
        String operatorName;
        int signalPct = 0;
        if (connected) {
            signalPct = gsmManager.getSignalPercent();
            operatorName = gsmManager.getOperator();
        }

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            deviceState.gprs_connected = connected;
            deviceState.registered_2g = registered;
            
            if (connected) {
                deviceState.signal_percent = signalPct;
                strncpy(deviceState.network_name, operatorName.c_str(), sizeof(deviceState.network_name));
            }
            xSemaphoreGive(stateMutex);
        }

        // Periodically update battery voltage (every 60s)
        // Moved OUTSIDE the mutex to prevent blocking GPS/Display tasks (CBC takes ~1s)
        static uint32_t lastBattCheck = 0;
        if (millis() - lastBattCheck > 60000 || lastBattCheck == 0) {
            int pct = -1;
            serverComm.closeSocket(); // ensure no TCP state before AT+CBC
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

                    // ── Refresh local ephemeris cache (rate-limited) ──
                    // Gated on a strong fix (≥ 6 sats) to avoid wasting flash
                    // writes on partial constellations.
                    static uint32_t lastEphPoll = 0;
                    bool   fixGood = false;
                    uint32_t nowEpoch = 0;
                    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        fixGood  = deviceState.gps_fix && deviceState.satellites >= 6;
                        nowEpoch = deviceState.utc_epoch;
                        xSemaphoreGive(stateMutex);
                    }
                    bool due = (lastEphPoll == 0)
                            || (millis() - lastEphPoll > EPH_POLL_INTERVAL_MS);
                    if (fixGood && due) {
                        gpsManager.pollEphemerides();
                        // Replies trickle in over ~1 s at 38 400 baud; 2 s
                        // is a safe ceiling. update() does the capturing.
                        vTaskDelay(pdMS_TO_TICKS(2000));
                        gpsManager.saveEphemeridesToNVS(nowEpoch);
                        lastEphPoll = millis();
                    }
                }
            }
        } else {
            gsmManager.ensureConnection();
        }

        // ── SMS polling (rate-limited) ──
        if (connected && lastSmsPoll != 0 && (millis() - lastSmsPoll > 30000)) {
            serverComm.closeSocket();  // release TCP before raw AT
            smsManager.pollSms();
            lastSmsPoll = millis();
            
            // Copy SMS data into shared state
            if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                smsManager.fillState(deviceState);
                xSemaphoreGive(stateMutex);
            }
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
                        if (displayManager.getCurrentScreen() == 7) {
                            // Cycle sub-screen: 0 (SMS list) -> 1 (SMS reader) -> 0
                            if (deviceState.msg_sub_screen == 0 && deviceState.sms_count > 0) {
                                deviceState.msg_sub_screen = 1;  // open reader
                                deviceState.msg_selected_sms = 0;
                                
                                smsManager.markRead(0);
                                if (deviceState.sms_unread[0]) {
                                    deviceState.sms_unread[0] = false;
                                    if (deviceState.sms_unread_count > 0) deviceState.sms_unread_count--;
                                }
                            } else if (deviceState.msg_sub_screen == 1) {
                                // Next SMS or switch back to list
                                if (deviceState.msg_selected_sms + 1 < deviceState.sms_count) {
                                    deviceState.msg_selected_sms++;
                                    
                                    smsManager.markRead(deviceState.msg_selected_sms);
                                    if (deviceState.sms_unread[deviceState.msg_selected_sms]) {
                                        deviceState.sms_unread[deviceState.msg_selected_sms] = false;
                                        if (deviceState.sms_unread_count > 0) deviceState.sms_unread_count--;
                                    }
                                } else {
                                    deviceState.msg_sub_screen = 0;  // back to SMS list
                                }
                            } else {
                                deviceState.msg_sub_screen = 0;
                            }
                        }
                        // Trigger Connectivity Test (Ping)
                        else if (displayManager.getCurrentScreen() == 4) {
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
                        if (displayManager.getCurrentScreen() == 7 && deviceState.msg_sub_screen == 1) {
                            deviceState.msg_sub_screen = 0; // back to list
                        } else {
                            // Enter deep sleep
                            powerManager.enterDeepSleep(0); // 0 = no timed wake
                        }
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
