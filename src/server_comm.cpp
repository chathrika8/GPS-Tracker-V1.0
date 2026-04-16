#include "server_comm.h"
#include "config.h"
#include <ArduinoJson.h>

ServerComm serverComm;

void ServerComm::begin() {}

int ServerComm::sendBatch(int maxCount) {
    GPSPacket packets[UPLINK_BATCH_SIZE_IDLE];
    int count = packetBuffer.readBatch(packets, maxCount);
    if (count == 0) return 0;

    String json = buildBatchJSON(packets, count);
    bool ok = postJSON(PROXY_HOST, PROXY_PATH, json);

    // Always remove — we don't retry. If the send failed the data is gone,
    // which is preferable to replaying stale positions after a long outage.
    packetBuffer.removeBatch(count);
    return ok ? count : 0;
}

String ServerComm::buildBatchJSON(GPSPacket* packets, int count) {
    JsonDocument doc;
    doc["device_id"] = DEVICE_ID;
    doc["fw_ver"]    = FW_VERSION;
    doc["buf_count"] = packetBuffer.count();

    JsonArray arr = doc["packets"].to<JsonArray>();
    for (int i = 0; i < count; i++) {
        JsonObject p = arr.add<JsonObject>();
        p["ts"]         = packets[i].timestamp;
        p["lat"]        = serialized(String(packets[i].latitude,  6));
        p["lon"]        = serialized(String(packets[i].longitude, 6));
        p["alt_m"]      = packets[i].altitude_m;
        p["speed_kmh"]  = packets[i].speed_kmh;
        p["course"]     = packets[i].course;
        p["sats"]       = packets[i].satellites;
        p["hdop"]       = packets[i].hdop;
        p["gsm_signal"] = packets[i].gsm_signal;
        p["batt_v"]     = packets[i].battery_v;
    }

    String output;
    serializeJson(doc, output);
    return output;
}

bool ServerComm::postJSON(const char* host, const char* endpoint, const String& body) {
    TinyGsmClient& client = gsmManager.getClient();

    _tcpStage   = "CONNECTING";
    _tcpHdrSent = 0;
    _tcpBodSent = 0;

    if (!client.connect(host, 80)) {
        _lastHttpCode = -1;
        _lastResponse = "CONN FAILED";
        _tcpStage     = "CONN FAIL";
        return false;
    }
    _tcpStage = "CONN OK";

    // Brief pause so the TCP window is open before we start writing
    delay(100);

    // Build and send the HTTP request. Headers go as a single print() call
    // to reduce the number of AT+CIPSEND roundtrips.
    String headers;
    headers.reserve(256);
    headers  = "POST ";      headers += endpoint;       headers += " HTTP/1.1\r\n";
    headers += "Host: ";     headers += host;           headers += "\r\n";
    headers += "User-Agent: ESP32-GPS-Tracker/1.0\r\n";
    headers += "Content-Type: application/json\r\n";
    headers += "Content-Length: "; headers += body.length(); headers += "\r\n";
    headers += "Connection: close\r\n\r\n";

    _tcpStage   = "SEND HDR";
    _tcpHdrSent = client.print(headers);

    _tcpBodLen  = body.length();
    _tcpStage   = "SEND BOD";
    _tcpBodSent = client.print(body);

    // Don't wait for a response — close immediately after the body is written.
    // The Cloudflare Worker will process it asynchronously.
    client.stop();

    bool sent     = (_tcpBodSent > 0);
    _lastHttpCode = 0;
    _lastResponse = sent ? "SENT" : "SEND FAIL";
    _tcpStage     = sent ? "DONE" : "SEND FAIL";
    return sent;
}

bool ServerComm::testConnectivity() {
    TinyGsmClient& client = gsmManager.getClient();
    _lastResponse = "PINGING...";

    // api.ipify.org responds to plain HTTP — useful as a GPRS smoke test
    if (!client.connect("api.ipify.org", 80)) {
        _lastHttpCode = -1;
        _lastResponse = "PING FAILED";
        return false;
    }

    client.print("GET / HTTP/1.1\r\nHost: api.ipify.org\r\nConnection: close\r\n\r\n");

    unsigned long deadline = millis() + 5000;
    while (!client.available() && millis() < deadline) delay(10);

    if (client.available()) {
        // Skip response headers to get to the bare IP body
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
