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

    // AGPS
    bool     agps_injected;     // true once UBX-AID-INI / MGA bytes were sent
    uint16_t agps_bytes;        // bytes streamed from AssistNow on last attempt
};

class GPSManager {
public:
    void begin();
    void update();
    void fillState(DeviceState& state);

    // Inject UTC + last-known position via UBX-AID-INI. Time epoch == 0 means
    // "skip time" (no GSM time sync yet). Lat/lon == 0 means "skip position".
    void injectAidIni(uint32_t utcEpoch, double lat, double lon, float altM);

    // Stream a raw UBX/MGA blob (typically returned by the AssistNow Worker)
    // straight to the GPS UART. The bytes are forwarded unmodified.
    void injectAssistNowBlob(const uint8_t* data, size_t len);

    // Persist the most recent valid fix to NVS so the next boot can seed
    // UBX-AID-INI without waiting for a fresh fix.
    void saveLastPositionToNVS(double lat, double lon, float altM, uint32_t epoch);
    bool loadLastPositionFromNVS(double* lat, double* lon, float* altM, uint32_t* epoch);

private:
    TinyGPSPlus  _gps;
    HardwareSerial* _serial = nullptr;

    void configureUBX();
    void sendUBX(const uint8_t* msg, size_t len);
    void appendUbxChecksum(uint8_t* buf, size_t payloadStart, size_t payloadEnd);
    const char* courseToCompass(double course);
    uint32_t computeUnixEpoch(int year, int month, int day, int hour, int minute, int second);
};

extern GPSManager gpsManager;
