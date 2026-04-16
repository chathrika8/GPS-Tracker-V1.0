#include "display_manager.h"
#include "packet_buffer.h"
#include "config.h"

DisplayManager displayManager;

void DisplayManager::begin() {
    _display = new Adafruit_SH1107(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

    if (!_display->begin(OLED_ADDR, true)) {
        Serial.println("[DISP] SH1107 init failed!");
        return;
    }

    // Increase I2C clock speed to 400kHz (Fast Mode) for much higher framerate
    Wire.setClock(400000);

    _display->setRotation(2); // Flipped 180 degrees for chassis mount
    _display->clearDisplay();
    _display->setTextColor(SH110X_WHITE);
    // --- Premium Boot Animation ---
    _display->setTextSize(2);
    const char line1[] = "GPS";
    const char line2[] = "TRACKER";

    // Typing effect for "GPS"
    for(int i = 0; i < 3; i++) {
        _display->setCursor(46 + (i * 12), 30);
        _display->print(line1[i]);
        _display->display();
        delay(50);
    }
    // Typing effect for "TRACKER"
    for(int i = 0; i < 7; i++) {
        _display->setCursor(22 + (i * 12), 55);
        _display->print(line2[i]);
        _display->display();
        delay(50);
    }
    // Sweeping lines expanding from center
    for(int w = 0; w <= 128; w += 8) {
        _display->drawLine(64 - w/2, 20, 64 + w/2, 20, SH110X_WHITE);
        _display->drawLine(64 - w/2, 80, 64 + w/2, 80, SH110X_WHITE);
        _display->display();
        delay(20);
    }

    // Reveal final text
    _display->setTextSize(1);
    _display->setCursor(45, 90);
    _display->print(FW_VERSION);
    
    _display->display();

    delay(1500); // Hold logo on screen

    Serial.println("[DISP] SH1107 ready.");
}

void DisplayManager::render(const DeviceState& state) {
    if (!_displayOn || !_display) return;

    _display->clearDisplay();

    switch (_currentScreen) {
        case 0: renderMainScreen(state);   break;
        case 1: renderGSMScreen(state);    break;
        case 2: renderCoordsScreen(state); break;
        case 3: renderSystemScreen(state); break;
        case 4: renderUplinkScreen(state); break;
        case 5: renderTcpDebugScreen(state); break;
        case 6: renderModuleDiagsScreen(state); break;
    }

    _display->display();
}

void DisplayManager::nextScreen() {
    _currentScreen = (_currentScreen + 1) % NUM_SCREENS;
}

void DisplayManager::toggleDisplay() {
    _displayOn = !_displayOn;
    if (_display) {
        if (_displayOn) {
            _display->oled_command(SH110X_DISPLAYON);
        } else {
            _display->oled_command(SH110X_DISPLAYOFF);
        }
    }
}

// ─────────────────────────────────────────────
// Screen 0: Main Dashboard
// ─────────────────────────────────────────────
void DisplayManager::renderMainScreen(const DeviceState& state) {
    drawHeader(state);

    // Altitude + Satellites
    _display->setTextSize(1);
    _display->setCursor(0, 18);
    _display->printf("%05.0f ft", state.altitude_ft);
    _display->setCursor(90, 18);
    _display->printf("%2d SAT", state.satellites);

    // Speed (KPH) — huge digits
    _display->setTextSize(4);
    _display->setCursor(5, 40);
    _display->printf("%3.0f", state.speed_kmh); // %3.0f prevents leading zeros

    // Compass + KPH label
    _display->setTextSize(1);
    _display->setCursor(90, 42);
    _display->printf("C:%s", state.compass);
    
    _display->setTextSize(2);
    _display->setCursor(90, 56);
    _display->print("KPH");

    // Coordinates (Single line, Centered)
    char coordStr[32];
    snprintf(coordStr, sizeof(coordStr), "%.5f / %.5f", state.latitude, state.longitude);
    
    _display->setTextSize(1);
    int strWidth = strlen(coordStr) * 6;
    _display->setCursor((128 - strWidth) / 2, 96);
    _display->print(coordStr);

    drawFooter(state);
}

// ─────────────────────────────────────────────
// Screen 1: GSM Info
// ─────────────────────────────────────────────
void DisplayManager::renderGSMScreen(const DeviceState& state) {
    _display->setTextSize(1);
    _display->setCursor(0, 0);
    _display->println("GSM STATUS");
    _display->drawLine(0, 10, 128, 10, SH110X_WHITE);

    _display->setCursor(0, 16);
    _display->print("Signal: ");
    drawSignalBars(56, 16, state.signal_percent);
    _display->setCursor(90, 16);
    _display->printf("%d%%", state.signal_percent);

    _display->setCursor(0, 30);
    _display->printf("Network: %.12s", state.network_name);

    _display->setCursor(0, 42);
    _display->printf("2G: %s", state.registered_2g ? "YES" : "NO");

    _display->setCursor(0, 54);
    _display->printf("GPRS: %s", state.gprs_connected ? "Connected" : "Disconn.");

    _display->setCursor(0, 72);
    _display->printf("Buffer: %lu pkts", (unsigned long)state.buffer_count);

    _display->setCursor(0, 84);
    _display->printf("WiFi: %s", state.wifi_enabled ? "ON" : "OFF");
    _display->setCursor(64, 84);
    _display->printf("BLE: %s", state.ble_enabled ? "ON" : "OFF");

    _display->setCursor(0, 118);
    _display->print("[A] Back  [B] ForceTX");
}

// ─────────────────────────────────────────────
// Screen 2: Coordinates
// ─────────────────────────────────────────────
void DisplayManager::renderCoordsScreen(const DeviceState& state) {
    _display->setTextSize(1);
    _display->setCursor(0, 0);
    _display->println("COORDINATES");
    _display->drawLine(0, 10, 128, 10, SH110X_WHITE);

    _display->setCursor(0, 18);
    _display->printf("Lat:  %.6f", state.latitude);

    _display->setCursor(0, 30);
    _display->printf("Lon:  %.6f", state.longitude);

    _display->setCursor(0, 46);
    _display->printf("Alt:  %.1f m", state.altitude_m);
    _display->setCursor(0, 58);
    _display->printf("      %.0f ft", state.altitude_ft);

    _display->setCursor(0, 74);
    _display->printf("HDOP: %.1f", state.hdop);

    _display->setCursor(0, 86);
    _display->printf("Fix:  %s", state.gps_fix ? "YES" : "NO");

    _display->setCursor(0, 100);
    _display->printf("Sats: %d", state.satellites);
}

// ─────────────────────────────────────────────
// Screen 3: System Info
// ─────────────────────────────────────────────
void DisplayManager::renderSystemScreen(const DeviceState& state) {
    _display->setTextSize(1);
    _display->setCursor(0, 0);
    _display->println("SYSTEM INFO");
    _display->drawLine(0, 10, 128, 10, SH110X_WHITE);

    uint32_t upH = state.uptime_sec / 3600;
    uint32_t upM = (state.uptime_sec % 3600) / 60;
    uint32_t upS = state.uptime_sec % 60;

    _display->setCursor(0, 18);
    _display->printf("Uptime: %02lu:%02lu:%02lu", upH, upM, upS);

    _display->setCursor(0, 30);
    _display->printf("Batt:  %.2fV  %d%%", state.battery_voltage, state.battery_percent);

    _display->setCursor(0, 42);
    _display->printf("FW:    %s", state.fw_version);

    _display->setCursor(0, 58);
    if (state.ota_available) {
        _display->printf("OTA:   %s avail!", state.ota_version);
    } else {
        _display->print("OTA:   Up to date");
    }

    _display->setCursor(0, 74);
    _display->printf("Buffer: %lu pkts", (unsigned long)state.buffer_count);

    _display->setCursor(0, 86);
    _display->printf("Sched: %s", state.schedule_active ? "ACTIVE" : "idle");

    _display->setCursor(0, 100);
    _display->printf("ID: %s", DEVICE_ID);
}

// ─────────────────────────────────────────────
// Screen 4: Uplink Diagnostics
// ─────────────────────────────────────────────
void DisplayManager::renderUplinkScreen(const DeviceState& state) {
    _display->setTextSize(1);
    _display->setCursor(0, 0);
    _display->println("UPLINK DIAGS");
    _display->drawLine(0, 10, 128, 10, SH110X_WHITE);

    _display->setCursor(0, 18);
    _display->printf("Last HTTP: %d", state.last_http_code);

    _display->setCursor(0, 30);
    _display->printf("Resp: %.22s", state.last_response);

    _display->setCursor(0, 46);
    if (state.last_uplink_time == 0) {
        _display->print("Last UL: Never");
    } else {
        uint32_t ago = (millis() - state.last_uplink_time) / 1000;
        _display->printf("Last UL: %lu s ago", (unsigned long)ago);
    }

    _display->setCursor(0, 58);
    _display->printf("Total: %lu pkts", (unsigned long)state.total_packets_sent);

    _display->setCursor(0, 74);
    _display->printf("Buf:   %lu pkts", (unsigned long)state.buffer_count);

    _display->setCursor(0, 90);
    _display->printf("GPRS:  %s", state.gprs_connected ? "Connected" : "Disconn.");

    _display->setCursor(0, 118);
    _display->print("[A] Next  [B] PingTest");
}

// ─────────────────────────────────────────────
// Screen 5: TCP Debug
// ─────────────────────────────────────────────
void DisplayManager::renderTcpDebugScreen(const DeviceState& state) {
    _display->setTextSize(1);
    _display->setCursor(0, 0);
    _display->print("TCP DEBUG");
    _display->drawLine(0, 10, 128, 10, SH110X_WHITE);

    _display->setCursor(0, 16);
    _display->printf("Stage: %.11s", state.tcp_stage);

    _display->setCursor(0, 28);
    _display->printf("Hdr: %d B sent", state.tcp_hdr_sent);

    _display->setCursor(0, 40);
    _display->printf("Bod: %d/%d B", state.tcp_bod_sent, state.tcp_bod_len);

    _display->setCursor(0, 56);
    _display->printf("HTTP: %d", state.last_http_code);

    _display->setCursor(0, 68);
    _display->printf("Resp: %.22s", state.last_response);

    _display->setCursor(0, 84);
    _display->printf("GPRS: %s", state.gprs_connected ? "OK" : "NO");

    _display->setCursor(0, 96);
    _display->printf("Host: %.22s", PROXY_HOST);

    _display->setCursor(0, 118);
    _display->print("[A] Next");
}

// ─────────────────────────────────────────────
// Screen 6: Module Diagnostics
// ─────────────────────────────────────────────
void DisplayManager::renderModuleDiagsScreen(const DeviceState& state) {
    _display->setTextSize(1);
    _display->setCursor(0, 0);
    _display->print("MODULE DIAGS");
    _display->drawLine(0, 10, 128, 10, SH110X_WHITE);

    _display->setCursor(0, 16);
    _display->printf("WiFi: %s", state.wifi_enabled ? "ON" : "OFF");
    
    _display->setCursor(0, 28);
    _display->printf("SSID: %.14s", state.wifi_ssid[0] != '\0' ? state.wifi_ssid : "None");
    
    _display->setCursor(0, 40);
    _display->printf("RSSI: %d dBm", state.wifi_rssi);
    
    _display->setCursor(0, 56);
    _display->printf("GPRS: %s", state.gprs_connected ? "ON" : "OFF");

    _display->setCursor(0, 68);
    _display->printf("BLE:  %s", state.ble_enabled ? "ON" : "OFF");
    
    _display->setCursor(0, 118);
    if (state.wifi_enabled) {
        _display->print("[A] Next  [B] OFF WIFI");
    } else {
        _display->print("[A] Next  [B] ON  WIFI");
    }
}

// ── Helpers ──

void DisplayManager::drawHeader(const DeviceState& state) {
    _display->setTextSize(1);
    
    // Signal Bars (Animate until fully connected to GPRS)
    bool animating = !state.gprs_connected;
    drawSignalBars(0, 0, state.signal_percent, animating);
    
    // 2G Symbol Display
    _display->setCursor(26, 0);
    if (!state.gprs_connected) {
        // Blinking 2G icon while negotiating network attach
        if ((millis() / 500) % 2 == 0) {
            _display->print("2G");
        }
    } else {
        // Solid 2G icon when attached to GPRS network
        _display->print("2G");
    }

    // GPRS and Upload Animation
    _display->setCursor(54, 0);
    if (state.gprs_connected) {
        if (state.is_uploading) {
            // Blinking "LIVE" indicator
            int frame = (millis() / 500) % 2;
            if (frame == 0) _display->print("LIVE");
            else _display->print("    ");
        } else {
            _display->print("GPRS");
        }
    } else {
        _display->print("----");
    }

    // Battery percentage
    _display->setCursor(96, 0);
    if (state.battery_percent >= 0) {
        _display->printf("%d%%", state.battery_percent);
    } else {
        _display->print("---%");
    }
    
    _display->drawLine(0, 10, 128, 10, SH110X_WHITE);
}

void DisplayManager::drawFooter(const DeviceState& state) {
    _display->drawLine(0, 110, 128, 110, SH110X_WHITE);
    _display->setTextSize(1);
    
    // Time on LHS (with seconds)
    _display->setCursor(0, 116);
    _display->printf("%02d:%02d:%02d", state.hour, state.minute, state.second);
    
    // Date on RHS
    _display->setCursor(68, 116);
    _display->printf("%02d/%02d/%04d", state.month, state.day, state.year);
}

void DisplayManager::drawSignalBars(int x, int y, int percent, bool animate_loading) {
    int bars = animate_loading ? ((millis() / 500) % 5) : (percent / 25);
    for (int i = 0; i < bars; i++) {
        int h = 2 + i * 2;  // bar heights: 2, 4, 6, 8
        _display->fillRect(x + i * 6, y + 8 - h, 4, h, SH110X_WHITE);
    }
}
