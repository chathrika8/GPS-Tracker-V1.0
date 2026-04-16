#include "command_handler.h"
#include "gsm_manager.h"
#include "wifi_manager.h"
#include "schedule_manager.h"
#include "ota_manager.h"
#include "power_manager.h"
#include "config.h"
#include <ArduinoJson.h>

CommandHandler commandHandler;

void CommandHandler::begin() {
    Serial.println("[CMD] Ready.");
}

void CommandHandler::pollAndExecute() {
    // Strip any path component from PROXY_HOST to get just the hostname
    String host = String(PROXY_HOST);
    int slash = host.indexOf('/');
    if (slash > 0) host = host.substring(0, slash);

    String endpoint = "/rest/v1/commands?device_id=eq." + String(DEVICE_ID) + "&status=eq.pending";
    String response = httpGet(host.c_str(), endpoint.c_str());
    if (response.isEmpty()) return;

    JsonDocument doc;
    if (deserializeJson(doc, response)) {
        Serial.println("[CMD] Bad JSON in command response");
        return;
    }

    for (JsonObject cmd : doc["commands"].as<JsonArray>()) {
        const char* id      = cmd["id"];
        const char* command = cmd["command"];
        const char* params  = cmd["params"] | "";

        Serial.printf("[CMD] %s\n", command);
        executeCommand(command, params);

        // ACK — mark the command as executed on the server
        String ackBody = "{\"id\":\"" + String(id) + "\",\"status\":\"executed\"}";
        TinyGsmClient& client = gsmManager.getClient();
        if (client.connect(host.c_str(), 80)) {
            client.print("POST /rest/v1/command_acks HTTP/1.1\r\n");
            client.print("Host: "); client.print(host); client.print("\r\n");
            client.print("User-Agent: ESP32-GPS-Tracker/1.0\r\n");
            client.print("Content-Type: application/json\r\n");
            client.print("Content-Length: "); client.print(ackBody.length()); client.print("\r\n");
            client.print("Connection: close\r\n\r\n");
            client.print(ackBody);
            client.flush();
            client.stop();
            Serial.printf("[CMD] ACK sent: %s\n", id);
        }
    }
}

void CommandHandler::executeCommand(const char* command, const char* params) {
    if      (strcmp(command, "wifi_on")  == 0) { wifiManager.enable();  }
    else if (strcmp(command, "wifi_off") == 0) { wifiManager.disable(); }
    else if (strcmp(command, "reboot")   == 0) {
        Serial.println("[CMD] Rebooting...");
        ESP.restart();
    }
    else if (strcmp(command, "force_ota") == 0) {
        otaManager.checkAndUpdate();
    }
    else if (strcmp(command, "set_schedule") == 0) {
        scheduleManager.updateFromJSON(params);
    }
    else if (strcmp(command, "set_schedule_off") == 0) {
        scheduleManager.disable();
    }
    else if (strcmp(command, "sleep") == 0) {
        JsonDocument pdoc;
        if (deserializeJson(pdoc, params) == DeserializationError::Ok) {
            const char* wakeAt = pdoc["wake_at"] | "";
            const char* method = pdoc["method"]  | "rtc";

            // Keep radio on if the wake method relies on an incoming SMS/RI
            if (strcmp(method, "sms") == 0)
                gsmManager.setFunctionality(1);

            if (strlen(wakeAt) > 0)
                gsmManager.setAlarm(wakeAt);
        }
        powerManager.enterDeepSleep(0);
    }
    else if (strcmp(command, "ble_on")  == 0 ||
             strcmp(command, "ble_off") == 0) {
        Serial.println("[CMD] BLE control not yet implemented");
    }
    else {
        Serial.printf("[CMD] Unknown: %s\n", command);
    }
}

String CommandHandler::httpGet(const char* host, const char* endpoint) {
    TinyGsmClient& client = gsmManager.getClient();
    if (!client.connect(host, 80)) return "";

    client.print("GET "); client.print(endpoint); client.print(" HTTP/1.1\r\n");
    client.print("Host: "); client.print(host); client.print("\r\n");
    client.print("User-Agent: ESP32-GPS-Tracker/1.0\r\n");
    client.print("Connection: close\r\n\r\n");
    client.flush();

    unsigned long deadline = millis() + 10000;
    while (!client.available() && millis() < deadline) {
        delay(10);
        if (!client.connected()) break;
    }

    String body;
    bool bodyStarted = false;
    while (client.available()) {
        String line = client.readStringUntil('\n');
        if (!bodyStarted) {
            if (line == "\r" || line == "") bodyStarted = true;
        } else {
            body += line;
        }
    }

    client.stop();
    return body;
}
