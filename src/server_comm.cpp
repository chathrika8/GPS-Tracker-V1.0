#include "server_comm.h"
#include "config.h"
#include <ArduinoJson.h>

ServerComm serverComm;

void ServerComm::begin() {
    // Serial.println("[COMM] Server comm ready.");
}

int ServerComm::sendBatch(int maxCount) {
    GPSPacket packets[UPLINK_BATCH_SIZE_IDLE];
    int count = packetBuffer.readBatch(packets, maxCount);

    if (count == 0) return 0;

    String json = buildBatchJSON(packets, count);

    bool ok = postJSON(PROXY_HOST, PROXY_PATH, json);
    // Always discard — fire-and-forget. No retry queue, no backlog buildup.
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
        p["lat"]        = serialized(String(packets[i].latitude, 6));
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

    _tcpStage = "CONNECTING";
    _tcpHdrSent = 0;
    _tcpBodSent = 0;

    if (!client.connect(host, 80)) {
        _lastHttpCode = -1;
        _lastResponse = "CONN FAILED";
        _tcpStage = "CONN FAIL";
        return false;
    }

    _tcpStage = "CONN OK";

    // Small delay to let TCP handshake settle
    delay(100);

    // Send headers as a compact string, then body via write().
    String headers = "POST ";
    headers += endpoint;
    headers += " HTTP/1.1\r\n";
    headers += "Host: ";
    headers += host;
    headers += "\r\n";
    headers += "User-Agent: ESP32-GPS-Tracker/1.0\r\n";
    headers += "Content-Type: application/json\r\n";
    headers += "Content-Length: ";
    headers += String(body.length());
    headers += "\r\n";
    headers += "Connection: close\r\n";
    headers += "\r\n";

    _tcpStage = "SEND HDR";
    _tcpHdrSent = client.print(headers);

    _tcpBodLen  = body.length();
    _tcpStage = "SEND BOD";
    _tcpBodSent = client.print(body);

    // Pure fire-and-forget: close immediately after sending.
    // Success = bytes were written to the modem.
    client.stop();

    bool dataSent = (_tcpBodSent > 0);
    _lastHttpCode = 0;
    _lastResponse = dataSent ? "SENT" : "SEND FAIL";
    _tcpStage     = dataSent ? "DONE" : "SEND FAIL";
    return dataSent;
}

bool ServerComm::testConnectivity() {
    TinyGsmClient& client = gsmManager.getClient();
    
    _lastResponse = "PINGING...";
    
    // Try to connect to a reliable non-SSL server (ipify)
    if (!client.connect("api.ipify.org", 80)) {
        _lastHttpCode = -1;
        _lastResponse = "PING FAILED";
        return false;
    }

    client.print("GET / HTTP/1.1\r\nHost: api.ipify.org\r\nConnection: close\r\n\r\n");
    
    unsigned long timeout = millis() + 5000;
    while (!client.available() && millis() < timeout) delay(10);

    if (client.available()) {
        // Skip headers
        while(client.available()) {
            String line = client.readStringUntil('\n');
            if (line == "\r" || line == "") break;
        }
        // Read IP
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
