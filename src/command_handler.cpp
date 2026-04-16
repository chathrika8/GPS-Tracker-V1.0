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
    Serial.println("[CMD] Command handler ready.");
}

void CommandHandler::pollAndExecute() {
    // Extract host from PROXY_HOST
    String host = String(PROXY_HOST);
    int slashPos = host.indexOf('/');
    if (slashPos > 0) host = host.substring(0, slashPos);

    String endpoint = "/rest/v1/commands?device_id=eq." + String(DEVICE_ID) + "&status=eq.pending";
    String response = httpGet(host.c_str(), endpoint.c_str());

    if (response.length() == 0) return;

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, response);
    if (err) {
        Serial.printf("[CMD] JSON parse error: %s\n", err.c_str());
        return;
    }

    JsonArray commands = doc["commands"].as<JsonArray>();
    for (JsonObject cmd : commands) {
        const char* id      = cmd["id"];
        const char* command = cmd["command"];
        const char* params  = cmd["params"] | "";

        Serial.printf("[CMD] Executing: %s\n", command);
        executeCommand(command, params);

        // ACK the command
        String ackBody = "{\"id\":\"" + String(id) + "\",\"status\":\"executed\"}";
        // POST ack (reuse server_comm pattern)
        TinyGsmClient& client = gsmManager.getClient();
        if (client.connect(host.c_str(), 80)) {
            String ackEndpoint = "/rest/v1/command_acks";
            client.print("POST "); client.print(ackEndpoint); client.print(" HTTP/1.1\r\n");
            client.print("Host: "); client.print(host); client.print("\r\n");
            client.print("User-Agent: ESP32-GPS-Tracker/1.0\r\n");
            client.print("Content-Type: application/json\r\n");
            client.print("Content-Length: "); client.print(ackBody.length()); client.print("\r\n");
            client.print("Connection: close\r\n\r\n");
            client.print(ackBody);
            client.flush();
            client.stop();
            Serial.printf("[CMD] ACK sent for %s\n", id);
        }
    }
}

void CommandHandler::executeCommand(const char* command, const char* params) {
    if (strcmp(command, "wifi_on") == 0) {
        wifiManager.enable();
    }
    else if (strcmp(command, "wifi_off") == 0) {
        wifiManager.disable();
    }
    else if (strcmp(command, "ble_on") == 0) {
        // BLE enabling — placeholder
        Serial.println("[CMD] BLE ON — not yet implemented");
    }
    else if (strcmp(command, "ble_off") == 0) {
        Serial.println("[CMD] BLE OFF — not yet implemented");
    }
    else if (strcmp(command, "sleep") == 0) {
        // Parse wake_at from params
        JsonDocument pdoc;
        if (deserializeJson(pdoc, params) == DeserializationError::Ok) {
            const char* wakeAt = pdoc["wake_at"] | "";
            const char* method = pdoc["method"] | "rtc";

            if (strcmp(method, "sms") == 0) {
                // Keep SIM800L radio on for SMS wake
                gsmManager.setFunctionality(1);
            }

            if (strlen(wakeAt) > 0) {
                gsmManager.setAlarm(wakeAt);
            }
        }
        powerManager.enterDeepSleep(0);
    }
    else if (strcmp(command, "reboot") == 0) {
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
    else {
        Serial.printf("[CMD] Unknown command: %s\n", command);
    }
}

String CommandHandler::httpGet(const char* host, const char* endpoint) {
    TinyGsmClient& client = gsmManager.getClient();

    if (!client.connect(host, 80)) {
        return "";
    }

    client.print("GET "); client.print(endpoint); client.print(" HTTP/1.1\r\n");
    client.print("Host: "); client.print(host); client.print("\r\n");
    client.print("User-Agent: ESP32-GPS-Tracker/1.0\r\n");
    client.print("Connection: close\r\n\r\n");
    client.flush();

    // Wait for response
    unsigned long timeout = millis() + 10000;
    while (!client.available() && millis() < timeout) {
        delay(10);
        if (!client.connected()) break;
    }

    // Skip HTTP headers
    String body = "";
    bool headersDone = false;
    while (client.available()) {
        String line = client.readStringUntil('\n');
        if (!headersDone) {
            if (line == "\r" || line == "") headersDone = true;
        } else {
            body += line;
        }
    }

    client.stop();
    return body;
}
