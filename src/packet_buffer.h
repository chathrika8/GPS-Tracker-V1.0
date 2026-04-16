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
    uint32_t count();
    void clear();
    bool isFull();

private:
    static const char* BUFFER_FILE;
    static const uint32_t MAX_PACKETS = 2000;

    uint32_t  _readOffset  = 0;
    uint32_t  _writeOffset = 0;
    uint32_t  _count       = 0;
};

extern PacketBuffer packetBuffer;
