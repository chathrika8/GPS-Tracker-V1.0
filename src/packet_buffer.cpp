#include "packet_buffer.h"
#include "config.h"

PacketBuffer packetBuffer;

const char* PacketBuffer::BUFFER_FILE = "/gps_buffer.bin";

void PacketBuffer::begin() {
    if (!LittleFS.begin(true)) {
        Serial.println("[BUF] LittleFS mount failed! Attempting format...");
        if (!LittleFS.format()) {
            Serial.println("[BUF] LittleFS format failed!");
            return;
        }
        if (!LittleFS.begin(false)) {
            Serial.println("[BUF] LittleFS re-mount failed!");
            return;
        }
    }

    // Check if buffer file exists and get its size
    if (LittleFS.exists(BUFFER_FILE)) {
        File f = LittleFS.open(BUFFER_FILE, FILE_READ);
        if (f) {
            _count = f.size() / sizeof(GPSPacket);
            _writeOffset = f.size();
            f.close();
            Serial.printf("[BUF] Restored %u buffered packets\n", _count);
        }
    } else {
        Serial.println("[BUF] No existing buffer, starting fresh.");
    }
}

bool PacketBuffer::store(const GPSPacket& pkt) {
    if (_count >= MAX_PACKETS) {
        // Circular: remove oldest by rewriting
        // For simplicity, we truncate when full (can be improved to circular)
        Serial.println("[BUF] Buffer full! Oldest packets will be lost.");
        clear();
    }

    File f = LittleFS.open(BUFFER_FILE, FILE_APPEND);
    if (!f) {
        Serial.println("[BUF] Failed to open buffer for writing!");
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
    int read = 0;

    for (int i = 0; i < toRead; i++) {
        if (f.read((uint8_t*)&out[i], sizeof(GPSPacket)) == sizeof(GPSPacket)) {
            read++;
        } else {
            break;
        }
    }

    f.close();
    return read;
}

void PacketBuffer::removeBatch(int count) {
    if (count <= 0) return;

    _readOffset += count * sizeof(GPSPacket);
    _count -= count;

    // If buffer is fully drained, reset the file
    if (_count == 0) {
        LittleFS.remove(BUFFER_FILE);
        _readOffset  = 0;
        _writeOffset = 0;
    }
    // Compact file periodically if readOffset gets too large
    else if (_readOffset > sizeof(GPSPacket) * 500) {
        // Rewrite remaining packets to start of file
        File src = LittleFS.open(BUFFER_FILE, FILE_READ);
        if (!src) return;

        src.seek(_readOffset);

        // Read remaining into temp buffer
        size_t remaining = _count * sizeof(GPSPacket);
        uint8_t* tmp = (uint8_t*)malloc(remaining);
        if (!tmp) { src.close(); return; }

        src.read(tmp, remaining);
        src.close();

        // Rewrite
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

uint32_t PacketBuffer::count() {
    return _count;
}

void PacketBuffer::clear() {
    LittleFS.remove(BUFFER_FILE);
    _readOffset  = 0;
    _writeOffset = 0;
    _count       = 0;
    Serial.println("[BUF] Buffer cleared.");
}

bool PacketBuffer::isFull() {
    return _count >= MAX_PACKETS;
}
