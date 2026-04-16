#include "wifi_manager.h"
#include "config.h"

WiFiManager wifiManager;

void WiFiManager::begin() {
    // WiFi starts disabled — GPRS is the primary data path
    WiFi.mode(WIFI_OFF);
    _enabled = false;
    // Serial.println("[WIFI] Initialized (OFF by default — OTA only).");
}

void WiFiManager::enable() {
    if (_enabled) return;

    // Serial.println("[WIFI] Enabling...");
    WiFi.mode(WIFI_STA);
    connect();
    _enabled = true;
}

void WiFiManager::disable() {
    if (!_enabled) return;

    // Serial.println("[WIFI] Disabling...");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    _enabled = false;
}

void WiFiManager::connect() {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    // We shouldn't block here because this may be called while holding the stateMutex.
    // The ESP32 WiFi stack connects asynchronously in the background.
    // displayTask and fillState() will catch when WiFi.status() becomes WL_CONNECTED.
}

bool WiFiManager::isConnected() {
    return _enabled && WiFi.status() == WL_CONNECTED;
}

bool WiFiManager::isEnabled() {
    return _enabled;
}

int WiFiManager::getRSSI() {
    return WiFi.RSSI();
}

void WiFiManager::fillState(DeviceState& state) {
    state.wifi_enabled = _enabled;
    state.wifi_connected = (_enabled && WiFi.status() == WL_CONNECTED);
    if (state.wifi_connected) {
        state.wifi_rssi = WiFi.RSSI();
        strncpy(state.wifi_ssid, WiFi.SSID().c_str(), sizeof(state.wifi_ssid) - 1);
        state.wifi_ssid[sizeof(state.wifi_ssid) - 1] = '\0';
    } else {
        state.wifi_rssi = 0;
        state.wifi_ssid[0] = '\0';
    }
}
