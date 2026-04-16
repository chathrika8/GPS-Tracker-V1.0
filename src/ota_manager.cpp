#include "ota_manager.h"
#include "gsm_manager.h"
#include "wifi_manager.h"
#include "config.h"
#include <Update.h>
#include <ArduinoJson.h>

OTAManager otaManager;

void OTAManager::begin() {
    Serial.println("[OTA] Ready.");
}

void OTAManager::checkAndUpdate() {
    Serial.println("[OTA] Checking for updates...");

    if (checkGitHub()) return;    // GitHub is primary
    if (checkSupabase()) return;  // Supabase manifest is the fallback

    Serial.println("[OTA] Firmware is up to date.");
}

bool OTAManager::checkGitHub() {
    TinyGsmClient& client = gsmManager.getClient();

    if (!client.connect("api.github.com", 443)) {
        Serial.println("[OTA] Cannot reach api.github.com");
        return false;
    }

    String path = "/repos/" + String(GITHUB_REPO_OWNER) + "/"
                + String(GITHUB_REPO_NAME) + "/releases/latest";

    client.print("GET " + path + " HTTP/1.1\r\n");
    client.print("Host: api.github.com\r\n");
    client.print("User-Agent: ESP32-GPS-Tracker\r\n");
    if (strlen(GITHUB_TOKEN) > 0)
        client.print("Authorization: Bearer " + String(GITHUB_TOKEN) + "\r\n");
    client.print("Connection: close\r\n\r\n");

    unsigned long deadline = millis() + 15000;
    while (!client.available() && millis() < deadline) delay(10);

    bool bodyStarted = false;
    String body;
    while (client.available()) {
        String line = client.readStringUntil('\n');
        if (!bodyStarted) {
            if (line == "\r" || line == "") bodyStarted = true;
        } else {
            body += line;
        }
    }
    client.stop();

    if (body.isEmpty()) return false;

    JsonDocument doc;
    if (deserializeJson(doc, body) != DeserializationError::Ok) {
        Serial.println("[OTA] Failed to parse GitHub response");
        return false;
    }

    const char* tagName = doc["tag_name"];
    if (!tagName) return false;

    // Strip optional 'v' prefix so "v1.2.0" and "1.2.0" both work
    const char* version = (tagName[0] == 'v' || tagName[0] == 'V') ? tagName + 1 : tagName;

    if (strcmp(version, FW_VERSION) <= 0) {
        Serial.printf("[OTA] GitHub: %s — no update needed\n", version);
        return false;
    }

    strncpy(_availableVersion, version, sizeof(_availableVersion));
    _updateAvailable = true;

    // Find the firmware.bin asset in this release
    String firmwareUrl;
    for (JsonObject asset : doc["assets"].as<JsonArray>()) {
        if (String(asset["name"].as<String>()) == "firmware.bin") {
            firmwareUrl = asset["browser_download_url"].as<String>();
            break;
        }
    }

    if (firmwareUrl.isEmpty()) {
        Serial.println("[OTA] No firmware.bin in release assets");
        return false;
    }

    Serial.printf("[OTA] Downloading v%s...\n", version);
    return downloadAndFlash(firmwareUrl.c_str(), 0, "");
}

bool OTAManager::checkSupabase() {
    TinyGsmClient& client = gsmManager.getClient();

    String host = String(PROXY_HOST);
    int slash = host.indexOf('/');
    if (slash > 0) host = host.substring(0, slash);

    if (!client.connect(host.c_str(), 80)) return false;

    String endpoint = "/rest/v1/" + String(SUPABASE_TABLE) + "/ota/manifest";
    client.print("GET " + endpoint + " HTTP/1.1\r\n");
    client.print("Host: " + host + "\r\n");
    client.print("User-Agent: ESP32-GPS-Tracker/1.0\r\n");
    client.print("Connection: close\r\n\r\n");
    client.flush();

    unsigned long deadline = millis() + 15000;
    while (!client.available() && millis() < deadline) {
        delay(10);
        if (!client.connected()) break;
    }

    bool bodyStarted = false;
    String body;
    while (client.available()) {
        String line = client.readStringUntil('\n');
        if (!bodyStarted) {
            if (line == "\r" || line == "") bodyStarted = true;
        } else {
            body += line;
        }
    }
    client.stop();

    if (body.isEmpty()) return false;

    JsonDocument doc;
    if (deserializeJson(doc, body) != DeserializationError::Ok) return false;

    const char* version = doc["version"];
    if (!version || strcmp(version, FW_VERSION) <= 0) return false;

    strncpy(_availableVersion, version, sizeof(_availableVersion));
    _updateAvailable = true;

    return downloadAndFlash(doc["url"] | "", doc["size"] | 0, doc["sha256"] | "");
}

bool OTAManager::downloadAndFlash(const char* url, size_t expectedSize, const char* sha256) {
    // TODO: implement full OTA download and flash.
    //
    // Outline:
    //   1. Follow GitHub's redirect chain to the CDN URL
    //   2. Stream binary into Update.write() in ~4 KB chunks
    //   3. Call Update.end(true) and verify SHA-256 if provided
    //   4. ESP.restart() — bootloader handles slot swap automatically
    //
    // Blocked on: HTTP redirect handling over TinyGSM raw TCP.

    Serial.printf("[OTA] Download pending — URL: %s\n", url);
    return false;
}

bool OTAManager::isUpdateAvailable()     { return _updateAvailable; }
const char* OTAManager::getAvailableVersion() { return _availableVersion; }
