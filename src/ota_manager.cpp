#include "ota_manager.h"
#include "gsm_manager.h"
#include "wifi_manager.h"
#include "config.h"
#include <Update.h>
#include <ArduinoJson.h>

OTAManager otaManager;

void OTAManager::begin() {
    Serial.println("[OTA] Manager ready.");
}

void OTAManager::checkAndUpdate() {
    Serial.println("[OTA] Checking for updates...");

    // Try GitHub first (primary)
    if (checkGitHub()) {
        Serial.printf("[OTA] GitHub update available: %s\n", _availableVersion);
        // Download will happen inside checkGitHub if update found
        return;
    }

    // Fallback to Supabase manifest
    if (checkSupabase()) {
        Serial.printf("[OTA] Supabase update available: %s\n", _availableVersion);
        return;
    }

    Serial.println("[OTA] No updates available.");
}

bool OTAManager::checkGitHub() {
    // Use WiFi for faster download if available, otherwise GPRS
    TinyGsmClient& client = gsmManager.getClient();

    if (!client.connect("api.github.com", 443)) {
        Serial.println("[OTA] Cannot reach GitHub API");
        return false;
    }

    String path = "/repos/" + String(GITHUB_REPO_OWNER) + "/" +
                  String(GITHUB_REPO_NAME) + "/releases/latest";

    client.print("GET " + path + " HTTP/1.1\r\n");
    client.print("Host: api.github.com\r\n");
    client.print("User-Agent: ESP32-GPS-Tracker\r\n");
    if (strlen(GITHUB_TOKEN) > 0) {
        client.print("Authorization: Bearer " + String(GITHUB_TOKEN) + "\r\n");
    }
    client.print("Connection: close\r\n\r\n");

    // Wait for response
    unsigned long timeout = millis() + 15000;
    while (!client.available() && millis() < timeout) delay(10);

    // Skip headers
    bool headersDone = false;
    String body = "";
    while (client.available()) {
        String line = client.readStringUntil('\n');
        if (!headersDone) {
            if (line == "\r" || line == "") headersDone = true;
        } else {
            body += line;
        }
    }
    client.stop();

    if (body.length() == 0) return false;

    // Parse JSON — extract tag_name and asset URLs
    JsonDocument doc;
    if (deserializeJson(doc, body) != DeserializationError::Ok) {
        Serial.println("[OTA] Failed to parse GitHub response");
        return false;
    }

    const char* tagName = doc["tag_name"];
    if (!tagName) return false;

    // Strip 'v' prefix for comparison
    const char* version = tagName;
    if (version[0] == 'v' || version[0] == 'V') version++;

    // Simple version comparison (string-based; works for semver)
    if (strcmp(version, FW_VERSION) <= 0) {
        Serial.printf("[OTA] GitHub version %s is not newer than %s\n", version, FW_VERSION);
        return false;
    }

    strncpy(_availableVersion, version, sizeof(_availableVersion));
    _updateAvailable = true;

    // Find firmware.bin asset
    String firmwareUrl = "";
    String sha256Url   = "";
    JsonArray assets = doc["assets"].as<JsonArray>();
    for (JsonObject asset : assets) {
        String name = asset["name"].as<String>();
        String url  = asset["browser_download_url"].as<String>();
        if (name == "firmware.bin") firmwareUrl = url;
        if (name == "sha256.txt")  sha256Url = url;
    }

    if (firmwareUrl.length() == 0) {
        Serial.println("[OTA] No firmware.bin asset found in release");
        return false;
    }

    // Download and flash
    Serial.printf("[OTA] Downloading firmware v%s from GitHub...\n", version);
    return downloadAndFlash(firmwareUrl.c_str(), 0, "");
}

bool OTAManager::checkSupabase() {
    TinyGsmClient& client = gsmManager.getClient();

    // Extract host from PROXY_HOST
    String host = String(PROXY_HOST);
    int slashPos = host.indexOf('/');
    if (slashPos > 0) host = host.substring(0, slashPos);

    if (!client.connect(host.c_str(), 80)) return false;

    String endpoint = "/rest/v1/" + String(SUPABASE_TABLE) + "/ota/manifest"; // Example path
    client.print("GET "); client.print(endpoint); client.print(" HTTP/1.1\r\n");
    client.print("Host: "); client.print(host); client.print("\r\n");
    client.print("User-Agent: ESP32-GPS-Tracker/1.0\r\n");
    client.print("Connection: close\r\n\r\n");
    client.flush();

    unsigned long timeout = millis() + 15000;
    while (!client.available() && millis() < timeout) {
        delay(10);
        if (!client.connected()) break;
    }

    bool headersDone = false;
    String body = "";
    while (client.available()) {
        String line = client.readStringUntil('\n');
        if (!headersDone) {
            if (line == "\r" || line == "") headersDone = true;
        } else {
            body += line;
        }
    }
    client.stop();

    if (body.length() == 0) return false;

    JsonDocument doc;
    if (deserializeJson(doc, body) != DeserializationError::Ok) return false;

    const char* version = doc["version"];
    if (!version || strcmp(version, FW_VERSION) <= 0) return false;

    strncpy(_availableVersion, version, sizeof(_availableVersion));
    _updateAvailable = true;

    const char* url    = doc["url"];
    size_t      size   = doc["size"] | 0;
    const char* sha256 = doc["sha256"] | "";

    return downloadAndFlash(url, size, sha256);
}

bool OTAManager::downloadAndFlash(const char* url, size_t expectedSize, const char* sha256) {
    // For now, use the TinyGSM client to download the binary
    // A full implementation would handle chunked transfer, SHA256 verification,
    // and write to the OTA partition using the Update library.

    Serial.printf("[OTA] Starting firmware download from: %s\n", url);
    Serial.println("[OTA] NOTE: Full OTA download implementation pending — ");
    Serial.println("[OTA]       requires HTTP redirect handling for GitHub URLs.");

    // Placeholder for the actual OTA flash logic:
    //
    // 1. Connect to URL (handle redirects)
    // 2. Begin Update: Update.begin(expectedSize)
    // 3. Stream data:  Update.write(buf, len) in chunks
    // 4. Finalize:     Update.end(true)
    // 5. Verify SHA256 if provided
    // 6. Reboot:       ESP.restart()

    return false; // Return false until fully implemented
}

bool OTAManager::isUpdateAvailable() {
    return _updateAvailable;
}

const char* OTAManager::getAvailableVersion() {
    return _availableVersion;
}
