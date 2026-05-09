#include "command_handler.h"
#include "log.h"
#include "gsm_manager.h"
#include "server_comm.h"        // needed to close the keep-alive socket before we grab the client
#include "wifi_manager.h"
#include "schedule_manager.h"
#include "ota_manager.h"
#include "power_manager.h"
#include "config.h"
#include <ArduinoJson.h>

CommandHandler commandHandler;

void CommandHandler::begin() {
    LOG("[CMD] Ready.");
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
        LOG("[CMD] Bad JSON in command response");
        return;
    }

    // Bug fix 1: Supabase REST returns a plain JSON array [{...}], NOT an
    // object with a "commands" key.  Iterate the root array directly.
    for (JsonObject cmd : doc.as<JsonArray>()) {
        const char* id      = cmd["id"];
        const char* command = cmd["command"];
        const char* params  = cmd["params"] | "";

        LOG("[CMD] %s\n", command);
        executeCommand(command, params);

        // ACK — PATCH /rest/v1/commands?id=eq.<id>  { "status": "executed" }
        // Bug fix 2: the old code POSTed to /rest/v1/command_acks which does
        // not exist.  The correct PostgREST pattern is a PATCH on the row.
        // Bug fix 3: serverComm holds a keep-alive TinyGsmClient open;
        // commandHandler must close it first or both share the same modem
        // channel and corrupt each other's TCP stream.
        serverComm.closeSocket();   // release keep-alive before we grab the client

        String ackPath = "/rest/v1/commands?id=eq." + String(id);
        String ackBody = "{\"status\":\"executed\"}";
        httpPatch(host.c_str(), ackPath.c_str(), ackBody);
        LOG("[CMD] ACK sent: %s\n", id);
    }

    if (_pendingReboot) {
        delay(500);  // Give TCP time to flush the ACK
        ESP.restart();
    }
}

void CommandHandler::executeCommand(const char* command, const char* params) {
    if      (strcmp(command, "wifi_on")  == 0) { wifiManager.enable();  }
    else if (strcmp(command, "wifi_off") == 0) { wifiManager.disable(); }
    else if (strcmp(command, "reboot")   == 0) {
        LOG("[CMD] Rebooting...");
        _pendingReboot = true;
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
        LOG("[CMD] BLE control not yet implemented");
    }
    else {
        LOG("[CMD] Unknown: %s\n", command);
    }
}

// ─── Shared HTTP helpers ─────────────────────────────────────────────────────
// Both helpers open a fresh connection (Connection: close) because commandHandler
// intentionally operates outside the serverComm keep-alive lifecycle.

static String _rawRequest(const char* host,
                           const char* method,
                           const char* endpoint,
                           const String& body) {
    TinyGsmClient& client = gsmManager.getClient();
    if (!client.connect(host, 80)) return "";

    client.print(method); client.print(" "); client.print(endpoint); client.print(" HTTP/1.1\r\n");
    client.print("Host: "); client.print(host); client.print("\r\n");
    client.print("User-Agent: ESP32-GPS-Tracker/1.0\r\n");
    if (body.length() > 0) {
        client.print("Content-Type: application/json\r\n");
        client.print("Content-Length: "); client.print(body.length()); client.print("\r\n");
    }
    client.print("Connection: close\r\n\r\n");
    if (body.length() > 0) client.print(body);
    client.flush();

    unsigned long deadline = millis() + 10000;
    while (!client.available() && millis() < deadline) {
        delay(10);
        if (!client.connected()) break;
    }

    String resp;
    resp.reserve(512);
    bool bodyStarted = false;
    while (client.available()) {
        String line = client.readStringUntil('\n');
        if (!bodyStarted) {
            if (line == "\r" || line == "") bodyStarted = true;
        } else {
            resp += line;
        }
    }

    client.stop();
    return resp;
}

String CommandHandler::httpGet(const char* host, const char* endpoint) {
    return _rawRequest(host, "GET", endpoint, "");
}

void CommandHandler::httpPatch(const char* host, const char* endpoint, const String& body) {
    _rawRequest(host, "PATCH", endpoint, body);
}
