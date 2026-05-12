#pragma once

#include <LittleFS.h>
#include <Arduino.h>
#include <cstdint>

// Compact binary packet — ~48 bytes
struct GPSPacket {
    uint32_t timestamp;    // UTC epoch (seconds)
    double   latitude;
    double   longitude;
    float    altitude_m;
    float    speed_kmh;
    float    course;
    uint8_t  satellites;
    float    hdop;
    uint8_t  gsm_signal;   // 0-100
    float    battery_v;
} __attribute__((packed));

class PacketBuffer {
public:
    void begin();
    bool store(const GPSPacket& pkt);
    int  readBatch(GPSPacket* out, int maxCount);
    void removeBatch(int count);
    // Drop packets at the head of the buffer whose timestamp is strictly
    // older than cutoffEpoch. Returns the number dropped. No-op if the
    // buffer is empty. Packets are appended in chronological order, so we
    // can stop at the first fresh one.
    uint32_t dropOlderThan(uint32_t cutoffEpoch);
    uint32_t count();
    void clear();
    bool isFull();
    bool isHealthy() { return _healthy; }

private:
    static const char* BUFFER_FILE;
    static const char* COMPACT_TMP_FILE;
    static const uint32_t MAX_PACKETS = 2000;

    uint32_t  _readOffset  = 0;
    uint32_t  _writeOffset = 0;
    uint32_t  _count       = 0;
    bool      _healthy     = false;
    bool      _errLogged   = false; // suppress per-packet error spam

    // Move the live region to the front of the file using a small stack
    // buffer and an atomic temp-file rename. Avoids ever allocating the
    // whole remaining payload on the heap. Returns true on success.
    bool compactInPlace();
    void markUnhealthy(const char* reason);
};

extern PacketBuffer packetBuffer;
