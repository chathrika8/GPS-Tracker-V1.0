#pragma once

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// ── Shared state fields populated by GPS ──
struct DeviceState; // Forward declaration (full definition below)

// Full DeviceState — shared across all modules
struct DeviceState {
    // GPS
    double   latitude;
    double   longitude;
    double   altitude_m;
    double   altitude_ft;
    double   speed_kmh;
    double   speed_mph;
    double   course;
    int      satellites;
    double   hdop;
    char     compass[4];
    bool     gps_fix;
    uint32_t utc_epoch;
    uint32_t gps_chars_processed;

    // GSM
    int      signal_strength;   // raw rssi (0–31)
    int      signal_percent;    // mapped 0–100
    bool     gprs_connected;
    bool     registered_2g;
    char     network_name[20];
    bool     is_uploading;
    bool     gsm_initialized;

    // Wi-Fi
    bool     wifi_enabled;
    bool     wifi_connected;
    int      wifi_rssi;
    char     wifi_ssid[32];

    // BLE
    bool     ble_enabled;

    // System
    uint32_t uptime_sec;
    float    battery_voltage;
    int      battery_percent = -1;  // 0–100% from AT+CBC bcl field; -1 = not yet read
    uint8_t  current_screen;
    char     fw_version[16];

    // OTA
    bool     ota_available;
    char     ota_version[16];

    // Buffer
    uint32_t buffer_count;

    // Uplink Diagnostics
    int      last_http_code;
    char     last_response[32];
    uint32_t last_uplink_time;
    uint32_t total_packets_sent;
    bool     trigger_ping_test;

    // TCP Debug
    char     tcp_stage[12];
    uint16_t tcp_hdr_sent;
    uint16_t tcp_bod_sent;
    uint16_t tcp_bod_len;    // expected body size

    // Timestamps (from GPS)
    uint8_t  hour, minute, second;
    uint8_t  day, month;
    uint16_t year;

    // Schedule
    bool     schedule_active;   // currently in a tracking window?
};

class GPSManager {
public:
    void begin();
    void update();
    void fillState(DeviceState& state);

private:
    TinyGPSPlus  _gps;
    HardwareSerial* _serial = nullptr;

    void configureUBX();
    void sendUBX(const uint8_t* msg, size_t len);
    const char* courseToCompass(double course);
    uint32_t computeUnixEpoch(int year, int month, int day, int hour, int minute, int second);
};

extern GPSManager gpsManager;
