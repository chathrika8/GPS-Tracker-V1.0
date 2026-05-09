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

// Minimal stateful UBX frame parser. Runs side-by-side with TinyGPSPlus
// in update(): TinyGPSPlus ignores binary bytes, this class ignores the
// $GP… NMEA stream, so they coexist on the same UART without contention.
class UbxParser {
public:
    UbxParser() { reset(); }
    void onByte(uint8_t b);
    bool hasFullFrame() const { return _ready; }
    uint8_t  getCls() const   { return _cls; }
    uint8_t  getId()  const   { return _id;  }
    uint16_t getLen() const   { return _len; }
    // Copies up to *outLen bytes of the full frame (including UBX header
    // and checksum, total = 8 + payload length) into out, sets *outLen to
    // the actual frame length, and resets the parser ready for the next.
    void consumeFrame(uint8_t* out, size_t* outLen);

private:
    void reset();
    enum State { S_S1, S_S2, S_CLS, S_ID, S_LL, S_LH, S_PL, S_KA, S_KB };
    State    _st;
    uint8_t  _cls, _id;
    uint16_t _len, _idx;
    uint8_t  _ckA, _ckB;
    uint8_t  _expCkA;
    uint8_t  _buf[120];   // max AID-EPH frame = 8 header/checksum + 104 payload
    bool     _ready;
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

    // ── Local ephemeris cache ──
    // Send UBX-AID-EPH poll-all to the receiver. The replies (one per SV
    // with a valid ephemeris, plus one short reply per SV without) trickle
    // back over the next ~1 s and are captured by update() into _ephBuf.
    void pollEphemerides();
    // Commit captured frames + a UTC timestamp to NVS. Returns the count.
    int  saveEphemeridesToNVS(uint32_t nowEpoch);
    // Replay cached frames if they're younger than maxAgeSec. Returns the
    // number of frames pushed to the receiver, or 0 if nothing was replayed.
    int  replayEphemeridesFromNVS(uint32_t nowEpoch, uint32_t maxAgeSec);

private:
    TinyGPSPlus     _gps;
    HardwareSerial* _serial = nullptr;
    UbxParser       _ubx;

    bool      _capturingEph = false;
    uint32_t  _ephCaptureStart = 0;
    uint8_t   _ephBuf[3584];          // 32 SVs × (8 header + 104 payload)
    size_t    _ephLen = 0;
    uint8_t   _ephCount = 0;

    void configureUBX();
    void sendUBX(const uint8_t* msg, size_t len);
    void appendUbxChecksum(uint8_t* buf, size_t payloadStart, size_t payloadEnd);
    void onUbxFrameReady();
    const char* courseToCompass(double course);
    uint32_t computeUnixEpoch(int year, int month, int day, int hour, int minute, int second);
};

extern GPSManager gpsManager;
