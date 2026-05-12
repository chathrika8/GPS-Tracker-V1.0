#include "packet_buffer.h"
#include "log.h"
#include "config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

PacketBuffer packetBuffer;

const char* PacketBuffer::BUFFER_FILE      = "/gps_buffer.bin";
const char* PacketBuffer::COMPACT_TMP_FILE = "/gps_buffer.tmp";

// ── Failure handling ────────────────────────────────────────────────────────
// Any LittleFS operation can fail (flash wear-out, power glitch, FS corruption,
// out-of-space). When that happens we tip into an "unhealthy" state where all
// public methods become safe no-ops. clear() is the recovery path: it retries
// FS init and, on success, returns the buffer to a fresh healthy state.
// The rest of the firmware sees this as "buffer is always empty / always
// refusing writes" — never a crash, never an unbounded retry storm.

void PacketBuffer::markUnhealthy(const char* reason) {
    if (!_errLogged) {
        LOG("[BUF] Unhealthy: %s — buffering disabled\n", reason);
        _errLogged = true;
    }
    _healthy = false;
}

void PacketBuffer::begin() {
    // First attempt: mount, formatting on failure if the partition is fresh.
    if (!LittleFS.begin(true)) {
        LOG("[BUF] LittleFS mount failed — formatting...");
        if (!LittleFS.format() || !LittleFS.begin(false)) {
            markUnhealthy("LittleFS unrecoverable at boot");
            return;
        }
    }

    // Clean up any half-finished compaction from a prior session.
    LittleFS.remove(COMPACT_TMP_FILE);

    _readOffset  = 0;
    _writeOffset = 0;
    _count       = 0;

    if (LittleFS.exists(BUFFER_FILE)) {
        File f = LittleFS.open(BUFFER_FILE, FILE_READ);
        if (f) {
            size_t sz = f.size();
            f.close();

            // Truncate any trailing partial record from an interrupted write
            // so _count and the on-disk size always agree on a record boundary.
            size_t aligned = (sz / sizeof(GPSPacket)) * sizeof(GPSPacket);
            if (aligned != sz) {
                LOG("[BUF] Trimming %u stray bytes from buffer\n",
                    (unsigned)(sz - aligned));
                // Best-effort trim: re-open WRITE wipes it; we accept the loss
                // of one partial record rather than try to in-place truncate.
                LittleFS.remove(BUFFER_FILE);
                aligned = 0;
            }
            _count       = aligned / sizeof(GPSPacket);
            _writeOffset = aligned;
            LOG("[BUF] Resumed with %u packets\n", (unsigned)_count);
        }
    }

    _healthy   = true;
    _errLogged = false;
}

bool PacketBuffer::store(const GPSPacket& pkt) {
    if (!_healthy) return false;

    if (_count >= MAX_PACKETS) {
        // Buffer full — drop everything and start fresh. After a long outage
        // the old data is stale anyway; we'd rather keep capturing fresh
        // positions than freeze on a saturated flash buffer.
        LOG("[BUF] Full — clearing stale packets");
        clear();
        if (!_healthy) return false;  // clear may have unhealth'd us
    }

    File f = LittleFS.open(BUFFER_FILE, FILE_APPEND);
    if (!f) {
        markUnhealthy("open(APPEND) failed");
        return false;
    }

    size_t written = f.write((const uint8_t*)&pkt, sizeof(GPSPacket));
    f.close();

    if (written != sizeof(GPSPacket)) {
        markUnhealthy("short write");
        return false;
    }

    _count++;
    _writeOffset += sizeof(GPSPacket);
    return true;
}

int PacketBuffer::readBatch(GPSPacket* out, int maxCount) {
    if (!_healthy || _count == 0 || maxCount <= 0 || !out) return 0;

    File f = LittleFS.open(BUFFER_FILE, FILE_READ);
    if (!f) {
        markUnhealthy("open(READ) failed");
        return 0;
    }

    if (!f.seek(_readOffset)) {
        f.close();
        markUnhealthy("seek failed");
        return 0;
    }

    int toRead = ((int)_count < maxCount) ? (int)_count : maxCount;
    int got    = 0;
    for (int i = 0; i < toRead; i++) {
        if (f.read((uint8_t*)&out[i], sizeof(GPSPacket)) != sizeof(GPSPacket)) break;
        got++;
    }
    f.close();
    return got;
}

void PacketBuffer::removeBatch(int count) {
    if (!_healthy || count <= 0) return;
    if ((uint32_t)count > _count) count = (int)_count;

    _readOffset += (uint32_t)count * sizeof(GPSPacket);
    _count      -= (uint32_t)count;

    if (_count == 0) {
        if (!LittleFS.remove(BUFFER_FILE)) {
            // remove() failing is non-fatal — the file is just empty-ish;
            // store() will append and we'll trim on next boot.
            LOG("[BUF] remove() after drain failed (ignored)");
        }
        _readOffset  = 0;
        _writeOffset = 0;
        return;
    }

    // Bound the dead head so the file can't grow forever. Threshold of 500
    // packet-slots gives roughly 24 KB of slack — enough to amortise the
    // compaction cost without letting the file balloon to MAX_PACKETS×2.
    if (_readOffset > sizeof(GPSPacket) * 500) {
        if (!compactInPlace()) {
            // Compaction failure means we couldn't safely shrink the file.
            // Rather than retry every send (which would re-fail), wipe the
            // buffer so capture can keep going. Stale positions in flash
            // are worse than no positions.
            LOG("[BUF] Compaction failed — clearing to recover");
            clear();
        }
    }
}

bool PacketBuffer::compactInPlace() {
    if (!_healthy) return false;
    if (_count == 0) {
        LittleFS.remove(BUFFER_FILE);
        _readOffset  = 0;
        _writeOffset = 0;
        return true;
    }

    File src = LittleFS.open(BUFFER_FILE, FILE_READ);
    if (!src) {
        markUnhealthy("compact: open(src) failed");
        return false;
    }
    if (!src.seek(_readOffset)) {
        src.close();
        markUnhealthy("compact: seek failed");
        return false;
    }

    // Best-effort cleanup of any leftover temp from a prior crash.
    LittleFS.remove(COMPACT_TMP_FILE);

    File dst = LittleFS.open(COMPACT_TMP_FILE, FILE_WRITE);
    if (!dst) {
        src.close();
        markUnhealthy("compact: open(tmp) failed");
        return false;
    }

    // Small stack buffer — never allocate the full remaining payload, which
    // could be up to MAX_PACKETS * sizeof(GPSPacket) (~96 KB).
    uint8_t  buf[512];
    size_t   expected = _count * sizeof(GPSPacket);
    size_t   copied   = 0;
    bool     ioErr    = false;

    while (copied < expected) {
        size_t want = expected - copied;
        if (want > sizeof(buf)) want = sizeof(buf);

        int got = src.read(buf, want);
        if (got <= 0) { ioErr = true; break; }

        size_t wrote = dst.write(buf, (size_t)got);
        if (wrote != (size_t)got) { ioErr = true; break; }
        copied += (size_t)got;

        // Yield every few KB so the watchdog and other tasks keep running.
        if ((copied & 0xFFF) == 0) vTaskDelay(pdMS_TO_TICKS(1));
    }

    src.close();
    dst.close();

    if (ioErr || copied != expected) {
        LittleFS.remove(COMPACT_TMP_FILE);
        return false;
    }

    // Replace the live file atomically (well, as atomic as LittleFS gets).
    // If the rename leg fails, we've lost the original — fall back to a
    // clean slate rather than leaving the system in a broken state.
    if (!LittleFS.remove(BUFFER_FILE)) {
        LittleFS.remove(COMPACT_TMP_FILE);
        return false;
    }
    if (!LittleFS.rename(COMPACT_TMP_FILE, BUFFER_FILE)) {
        LittleFS.remove(COMPACT_TMP_FILE);
        _count = 0;
        _readOffset = 0;
        _writeOffset = 0;
        return false;
    }

    _readOffset  = 0;
    _writeOffset = expected;
    return true;
}

uint32_t PacketBuffer::dropOlderThan(uint32_t cutoffEpoch) {
    if (!_healthy || _count == 0) return 0;

    File f = LittleFS.open(BUFFER_FILE, FILE_READ);
    if (!f) {
        markUnhealthy("dropOlderThan: open failed");
        return 0;
    }
    if (!f.seek(_readOffset)) {
        f.close();
        markUnhealthy("dropOlderThan: seek failed");
        return 0;
    }

    uint32_t dropped = 0;
    // Cap the per-call work so a huge stale backlog can't hog the task.
    // Anything still stale will be picked up on the next send.
    const uint32_t MAX_DROP_PER_CALL = 256;

    while (_count > 0 && dropped < MAX_DROP_PER_CALL) {
        GPSPacket pkt;
        if (f.read((uint8_t*)&pkt, sizeof(pkt)) != sizeof(pkt)) break;
        // Stop at the first fresh packet — the rest are at least as new
        // because store() appends in chronological order.
        if (pkt.timestamp >= cutoffEpoch) break;
        _readOffset += sizeof(GPSPacket);
        _count--;
        dropped++;
    }
    f.close();

    if (dropped > 0) {
        LOG("[BUF] Dropped %u stale packets\n", (unsigned)dropped);
        if (_count == 0) {
            LittleFS.remove(BUFFER_FILE);
            _readOffset  = 0;
            _writeOffset = 0;
        }
    }
    return dropped;
}

uint32_t PacketBuffer::count()  { return _healthy ? _count : 0; }
bool     PacketBuffer::isFull() { return _healthy && _count >= MAX_PACKETS; }

void PacketBuffer::clear() {
    // Always try to clear, even if unhealthy — this is the recovery path.
    LittleFS.remove(BUFFER_FILE);
    LittleFS.remove(COMPACT_TMP_FILE);
    _readOffset  = 0;
    _writeOffset = 0;
    _count       = 0;

    // If we were unhealthy because of a transient FS error, try to come
    // back online now that we've reset state. Re-mounting is cheap and
    // idempotent when the FS is already mounted.
    if (!_healthy) {
        if (LittleFS.begin(false)) {
            _healthy   = true;
            _errLogged = false;
            LOG("[BUF] Recovered after clear");
        }
    }
    LOG("[BUF] Cleared");
}
