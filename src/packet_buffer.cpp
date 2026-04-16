#include "packet_buffer.h"
#include "config.h"

PacketBuffer packetBuffer;

const char* PacketBuffer::BUFFER_FILE = "/gps_buffer.bin";

void PacketBuffer::begin() {
    if (!LittleFS.begin(true)) {
        Serial.println("[BUF] LittleFS mount failed — formatting...");
        if (!LittleFS.format() || !LittleFS.begin(false)) {
            Serial.println("[BUF] LittleFS unrecoverable");
            return;
        }
    }

    if (LittleFS.exists(BUFFER_FILE)) {
        File f = LittleFS.open(BUFFER_FILE, FILE_READ);
        if (f) {
            _count       = f.size() / sizeof(GPSPacket);
            _writeOffset = f.size();
            f.close();
            Serial.printf("[BUF] Resumed with %u packets\n", _count);
        }
    }
}

bool PacketBuffer::store(const GPSPacket& pkt) {
    if (_count >= MAX_PACKETS) {
        // Buffer full — drop everything and start fresh. This is intentional:
        // after a very long outage the old data is stale anyway, and we'd
        // rather send fresh positions than flood the server with history.
        Serial.println("[BUF] Full — clearing stale packets");
        clear();
    }

    File f = LittleFS.open(BUFFER_FILE, FILE_APPEND);
    if (!f) {
        Serial.println("[BUF] Open for write failed");
        return false;
    }

    size_t written = f.write((const uint8_t*)&pkt, sizeof(GPSPacket));
    f.close();

    if (written == sizeof(GPSPacket)) {
        _count++;
        _writeOffset += sizeof(GPSPacket);
        return true;
    }
    return false;
}

int PacketBuffer::readBatch(GPSPacket* out, int maxCount) {
    if (_count == 0) return 0;

    File f = LittleFS.open(BUFFER_FILE, FILE_READ);
    if (!f) return 0;

    f.seek(_readOffset);

    int toRead = min((int)_count, maxCount);
    int read   = 0;
    for (int i = 0; i < toRead; i++) {
        if (f.read((uint8_t*)&out[i], sizeof(GPSPacket)) != sizeof(GPSPacket)) break;
        read++;
    }

    f.close();
    return read;
}

void PacketBuffer::removeBatch(int count) {
    if (count <= 0) return;

    _readOffset += count * sizeof(GPSPacket);
    _count      -= count;

    if (_count == 0) {
        // All packets sent — delete the file rather than leaving an empty one
        LittleFS.remove(BUFFER_FILE);
        _readOffset  = 0;
        _writeOffset = 0;
        return;
    }

    // Compact the file once the dead head grows beyond 500 packet-slots.
    // This bounds flash wear by avoiding unbounded file growth.
    if (_readOffset > sizeof(GPSPacket) * 500) {
        size_t remaining = _count * sizeof(GPSPacket);
        uint8_t* tmp = (uint8_t*)malloc(remaining);
        if (!tmp) return;

        File src = LittleFS.open(BUFFER_FILE, FILE_READ);
        if (!src) { free(tmp); return; }
        src.seek(_readOffset);
        src.read(tmp, remaining);
        src.close();

        File dst = LittleFS.open(BUFFER_FILE, FILE_WRITE);
        if (dst) {
            dst.write(tmp, remaining);
            dst.close();
        }
        free(tmp);

        _readOffset  = 0;
        _writeOffset = remaining;
    }
}

uint32_t PacketBuffer::count()  { return _count; }
bool     PacketBuffer::isFull() { return _count >= MAX_PACKETS; }

void PacketBuffer::clear() {
    LittleFS.remove(BUFFER_FILE);
    _readOffset  = 0;
    _writeOffset = 0;
    _count       = 0;
    Serial.println("[BUF] Cleared");
}
