#include "server_comm.h"
#include "config.h"
#include <ArduinoJson.h>

ServerComm serverComm;

void ServerComm::begin() {}

// ─────────────────────────────────────────────
// Public entry: send up to maxCount packets
// ─────────────────────────────────────────────
int ServerComm::sendBatch(int maxCount) {
    if (maxCount <= 0) return 0;

    // Size the on-stack array to the larger of the two batch caps so a single
    // call can handle either moving or idle batching.
    static const int CAP = (UPLINK_BATCH_SIZE_IDLE > UPLINK_BATCH_SIZE_MOVING)
                              ? UPLINK_BATCH_SIZE_IDLE
                              : UPLINK_BATCH_SIZE_MOVING;
    GPSPacket packets[CAP];
    int want = (maxCount > CAP) ? CAP : maxCount;

    int count = packetBuffer.readBatch(packets, want);
    if (count == 0) return 0;

    bool ok;
#if PAYLOAD_PROFILE == PAYLOAD_PROFILE_COMPACT
    // Worst case: 11 B header + 20 B/packet
    uint8_t buf[11 + UPLINK_BATCH_SIZE_IDLE * 20];
    size_t  len = buildBatchBinary(packets, count, buf, sizeof(buf));
    ok = postBytes(PROXY_HOST, PROXY_PATH,
                   "application/octet-stream", buf, len);
#else
    String json = buildBatchJSON(packets, count);
    ok = postBytes(PROXY_HOST, PROXY_PATH,
                   "application/json",
                   (const uint8_t*)json.c_str(), json.length());
#endif

    // Fire-and-forget: drop the batch from the buffer regardless of ok.
    // Replaying stale positions after an outage is worse than losing them.
    packetBuffer.removeBatch(count);
    return ok ? count : 0;
}

// ─────────────────────────────────────────────
// JSON encoder — short keys + array-of-arrays
// ─────────────────────────────────────────────
// Wire layout:
//   {"d":"GPS-001","f":"1.1.0","b":12,"p":[[ts,lat,lon,alt,sp,co,sa,hd,gs,bv]]}
//
// Field order matches buildBatchBinary so the Worker can share a schema.
String ServerComm::buildBatchJSON(GPSPacket* packets, int count) {
    JsonDocument doc;
    doc["d"] = DEVICE_ID;
    doc["f"] = FW_VERSION;
    doc["b"] = packetBuffer.count();

    JsonArray arr = doc["p"].to<JsonArray>();
    for (int i = 0; i < count; i++) {
        JsonArray r = arr.add<JsonArray>();
        r.add(packets[i].timestamp);
        r.add(serialized(String(packets[i].latitude,  5)));
        r.add(serialized(String(packets[i].longitude, 5)));
        r.add((int)packets[i].altitude_m);
        r.add(serialized(String(packets[i].speed_kmh, 1)));
        r.add((int)packets[i].course);
        r.add(packets[i].satellites);
        r.add(serialized(String(packets[i].hdop, 1)));
        r.add(packets[i].gsm_signal);
        r.add(serialized(String(packets[i].battery_v, 2)));
    }

    String output;
    serializeJson(doc, output);
    return output;
}

// ─────────────────────────────────────────────
// Binary encoder — 11 B header + 20 B/packet
// ─────────────────────────────────────────────
// Header (little-endian):
//   [0]    'G'                   magic
//   [1]    0x01                  version
//   [2..5] device_id_hash        FNV-1a 32-bit of DEVICE_ID
//   [6..9] buf_count             uint32, queue depth at send time
//   [10]   packet_count          uint8
// Per packet (little-endian, 20 B):
//   [0..3]   ts        uint32  UTC epoch
//   [4..7]   lat       int32   degrees × 1e7
//   [8..11]  lon       int32   degrees × 1e7
//   [12..13] alt       int16   metres
//   [14]     speed     uint8   km/h × 2  (0–127.5 km/h)
//   [15]     course    uint8   degrees / 2  (0–510°, wraps at 256→512)
//   [16]     sats      uint8
//   [17]     hdop      uint8   × 10  (0–25.5)
//   [18]     gsm       uint8   percent
//   [19]     batt      uint8   (V − 3.0) × 100  (3.00–5.55 V)
size_t ServerComm::buildBatchBinary(GPSPacket* packets, int count,
                                    uint8_t* out, size_t maxLen) {
    if (maxLen < 11u + (size_t)count * 20u) return 0;

    // FNV-1a hash of DEVICE_ID — stable, no allocation
    uint32_t hash = 2166136261UL;
    for (const char* p = DEVICE_ID; *p; p++) {
        hash ^= (uint8_t)*p;
        hash *= 16777619UL;
    }

    size_t i = 0;
    out[i++] = 'G';
    out[i++] = 0x01;
    out[i++] = (uint8_t)(hash);
    out[i++] = (uint8_t)(hash >> 8);
    out[i++] = (uint8_t)(hash >> 16);
    out[i++] = (uint8_t)(hash >> 24);
    uint32_t bufC = packetBuffer.count();
    out[i++] = (uint8_t)(bufC);
    out[i++] = (uint8_t)(bufC >> 8);
    out[i++] = (uint8_t)(bufC >> 16);
    out[i++] = (uint8_t)(bufC >> 24);
    out[i++] = (uint8_t)count;

    for (int k = 0; k < count; k++) {
        const GPSPacket& p = packets[k];

        uint32_t ts  = p.timestamp;
        int32_t  lat = (int32_t)(p.latitude  * 1e7);
        int32_t  lon = (int32_t)(p.longitude * 1e7);
        int16_t  alt = (int16_t)constrain((int)p.altitude_m, -32768, 32767);

        int spdQ = (int)(p.speed_kmh * 2.0f + 0.5f);
        if (spdQ < 0) spdQ = 0; else if (spdQ > 255) spdQ = 255;

        float crs = p.course;
        while (crs < 0)    crs += 360.0f;
        while (crs >= 360) crs -= 360.0f;
        uint8_t crsQ = (uint8_t)((int)(crs * 0.5f + 0.5f) & 0xFF);

        int hdopQ = (int)(p.hdop * 10.0f + 0.5f);
        if (hdopQ < 0) hdopQ = 0; else if (hdopQ > 255) hdopQ = 255;

        int battQ = (int)((p.battery_v - 3.0f) * 100.0f + 0.5f);
        if (battQ < 0) battQ = 0; else if (battQ > 255) battQ = 255;

        out[i++] = (uint8_t)(ts);
        out[i++] = (uint8_t)(ts >> 8);
        out[i++] = (uint8_t)(ts >> 16);
        out[i++] = (uint8_t)(ts >> 24);
        out[i++] = (uint8_t)(lat);
        out[i++] = (uint8_t)(lat >> 8);
        out[i++] = (uint8_t)(lat >> 16);
        out[i++] = (uint8_t)(lat >> 24);
        out[i++] = (uint8_t)(lon);
        out[i++] = (uint8_t)(lon >> 8);
        out[i++] = (uint8_t)(lon >> 16);
        out[i++] = (uint8_t)(lon >> 24);
        out[i++] = (uint8_t)(alt);
        out[i++] = (uint8_t)(alt >> 8);
        out[i++] = (uint8_t)spdQ;
        out[i++] = crsQ;
        out[i++] = p.satellites;
        out[i++] = (uint8_t)hdopQ;
        out[i++] = p.gsm_signal;
        out[i++] = (uint8_t)battQ;
    }

    return i;
}

// ─────────────────────────────────────────────
// TCP keep-alive: open once, reuse, recycle on age
// ─────────────────────────────────────────────
bool ServerComm::ensureSocket(const char* host, uint16_t port) {
    TinyGsmClient& client = gsmManager.getClient();

    bool aged = (millis() - _socketOpenedAt) > UPLINK_KEEPALIVE_MAX_AGE_MS;
    if (_socketOpen && client.connected() && !aged) {
        // Drain any leftover response bytes from the previous keep-alive
        // request so they don't accumulate in the modem's RX buffer.
        while (client.available()) client.read();
        return true;
    }

    if (_socketOpen) {
        client.stop();
        _socketOpen = false;
    }

    _tcpStage = "CONNECTING";
    if (!client.connect(host, port)) {
        _tcpStage     = "CONN FAIL";
        _lastHttpCode = -1;
        _lastResponse = "CONN FAILED";
        return false;
    }

    _socketOpen     = true;
    _socketOpenedAt = millis();
    _tcpStage       = "CONN OK";
    return true;
}

void ServerComm::closeSocket() {
    if (_socketOpen) {
        gsmManager.getClient().stop();
        _socketOpen = false;
    }
}

// ─────────────────────────────────────────────
// Single-write POST: headers + body in one TCP send
// ─────────────────────────────────────────────
bool ServerComm::postBytes(const char* host,
                           const char* endpoint,
                           const char* contentType,
                           const uint8_t* body,
                           size_t bodyLen) {
    _tcpHdrSent = 0;
    _tcpBodSent = 0;
    _tcpBodLen  = bodyLen;

    if (!ensureSocket(host, 80)) return false;

    TinyGsmClient& client = gsmManager.getClient();

    // Build the entire request — headers + body — into one buffer so TinyGSM
    // emits a single AT+CIPSEND. This roughly halves modem roundtrips per send.
    String req;
    req.reserve(160 + bodyLen);
    req  = "POST ";  req += endpoint; req += " HTTP/1.1\r\n";
    req += "Host: "; req += host;     req += "\r\n";
    req += "Content-Type: "; req += contentType; req += "\r\n";
    req += "Content-Length: "; req += (uint32_t)bodyLen; req += "\r\n";
    req += "Connection: keep-alive\r\n\r\n";

    size_t headerLen = req.length();
    // Append body bytes (binary-safe via concat with length)
    if (bodyLen > 0) req.concat((const char*)body, bodyLen);

    _tcpStage = "SEND";
    size_t written = client.write((const uint8_t*)req.c_str(), req.length());

    if (written < headerLen) {
        _tcpHdrSent = written;
        _tcpStage   = "SEND FAIL";
        // Socket likely broken — force a reconnect on next call
        client.stop();
        _socketOpen = false;
        _lastHttpCode = -2;
        _lastResponse = "WRITE FAIL";
        return false;
    }

    _tcpHdrSent = headerLen;
    _tcpBodSent = written - headerLen;

    // Don't block waiting for a response. The Worker processes asynchronously,
    // and we keep the socket open for the next batch (keep-alive).
    _lastHttpCode = 0;
    _lastResponse = "SENT";
    _tcpStage     = "DONE";
    return true;
}

// ─────────────────────────────────────────────
// Connectivity smoke test (manual trigger)
// ─────────────────────────────────────────────
bool ServerComm::testConnectivity() {
    // Always fresh-connect for the diagnostic so it doesn't reuse a stale
    // keep-alive socket that might be a false positive.
    closeSocket();

    TinyGsmClient& client = gsmManager.getClient();
    _lastResponse = "PINGING...";

    if (!client.connect("api.ipify.org", 80)) {
        _lastHttpCode = -1;
        _lastResponse = "PING FAILED";
        return false;
    }

    client.print("GET / HTTP/1.1\r\nHost: api.ipify.org\r\nConnection: close\r\n\r\n");

    unsigned long deadline = millis() + 5000;
    while (!client.available() && millis() < deadline) delay(10);

    if (client.available()) {
        while (client.available()) {
            String line = client.readStringUntil('\n');
            if (line == "\r" || line == "") break;
        }
        String ip = client.readString();
        ip.trim();
        _lastResponse = "IP: " + ip;
        _lastHttpCode = 200;
        client.stop();
        return true;
    }

    client.stop();
    _lastResponse = "PING NO RESP";
    return false;
}

// ─────────────────────────────────────────────
// AssistNow fetch over the Cloudflare Worker
// ─────────────────────────────────────────────
// The Worker is expected to GET the u-blox AssistNow Online URL server-side
// and stream the binary body back unmodified. We pull until the response
// closes (or buffer fills) and copy the body into out.
bool ServerComm::fetchAssistNow(uint8_t* out, size_t maxLen, size_t* outLen) {
    if (outLen) *outLen = 0;
    closeSocket();

    TinyGsmClient& client = gsmManager.getClient();
    if (!client.connect(PROXY_HOST, 80)) {
        _lastHttpCode = -1;
        _lastResponse = "AGPS CONN FAIL";
        return false;
    }

    String req;
    req.reserve(96);
    req  = "GET ";  req += ASSISTNOW_PATH; req += " HTTP/1.1\r\n";
    req += "Host: "; req += PROXY_HOST;    req += "\r\n";
    req += "Connection: close\r\n\r\n";
    client.print(req);

    // Wait up to 10 s for headers to start arriving — AssistNow can be slow.
    unsigned long deadline = millis() + 10000;
    while (!client.available() && millis() < deadline) delay(20);
    if (!client.available()) {
        client.stop();
        _lastHttpCode = -3;
        _lastResponse = "AGPS NO RESP";
        return false;
    }

    // Parse status line
    String statusLine = client.readStringUntil('\n');
    int code = 0;
    int sp = statusLine.indexOf(' ');
    if (sp > 0) code = statusLine.substring(sp + 1, sp + 4).toInt();
    _lastHttpCode = code;

    // Skip remaining headers
    while (client.connected() || client.available()) {
        if (!client.available()) { delay(10); continue; }
        String line = client.readStringUntil('\n');
        if (line == "\r" || line.length() <= 1) break;
    }

    if (code != 200) {
        client.stop();
        _lastResponse = "AGPS HTTP " + String(code);
        return false;
    }

    size_t total = 0;
    deadline = millis() + 10000;
    while ((client.connected() || client.available()) && total < maxLen) {
        if (!client.available()) {
            if (millis() > deadline) break;
            delay(10);
            continue;
        }
        int n = client.read(out + total, maxLen - total);
        if (n > 0) {
            total += n;
            deadline = millis() + 2000;  // refresh idle timeout per chunk
        }
    }

    client.stop();
    if (outLen) *outLen = total;
    _lastResponse = "AGPS " + String((unsigned)total) + "B";
    return total > 0;
}
