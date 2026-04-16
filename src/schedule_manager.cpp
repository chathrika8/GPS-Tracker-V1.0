#include "schedule_manager.h"
#include "config.h"
#include <Preferences.h>
#include <ArduinoJson.h>

ScheduleManager scheduleManager;
static Preferences schedPrefs;

void ScheduleManager::begin() {
    _enabled = SCHEDULE_ENABLED;
    loadFromNVS();

    // Serial.printf("[SCHED] Schedule %s, %d windows configured\n",
    //               _enabled ? "ENABLED" : "DISABLED", _windowCount);
}

void ScheduleManager::checkWindow(DeviceState& state) {
    if (!_enabled) {
        state.schedule_active = true;  // always active when schedule disabled
        return;
    }

    // Convert UTC time from GPS to local time
    int localMinutes = (state.hour * 60 + state.minute)
                     + (_tzOffsetHours * 60 + _tzOffsetMinutes);
    if (localMinutes < 0)    localMinutes += 1440;
    if (localMinutes >= 1440) localMinutes -= 1440;

    uint8_t localHour   = localMinutes / 60;
    uint8_t localMinute = localMinutes % 60;

    bool active = isInWindow(localHour, localMinute);
    state.schedule_active = active;

    static bool lastActive = true;
    if (active != lastActive) {
        // Serial.printf("[SCHED] Tracking window %s (local %02d:%02d)\n",
    //               active ? "ENTERED" : "EXITED", localHour, localMinute);
        lastActive = active;

        if (!active) {
            // TODO: Enter light sleep — GPS off, display off
            // Serial.println("[SCHED] Entering idle mode until next window.");
        }
    }
}

bool ScheduleManager::isInWindow(uint8_t hour, uint8_t minute) {
    int nowMinutes = hour * 60 + minute;

    for (uint8_t i = 0; i < _windowCount; i++) {
        int startMin = _windows[i].startHour * 60 + _windows[i].startMinute;
        int endMin   = _windows[i].endHour   * 60 + _windows[i].endMinute;

        if (endMin > startMin) {
            // Normal window (e.g. 06:00 - 09:00)
            if (nowMinutes >= startMin && nowMinutes < endMin) return true;
        } else {
            // Overnight window (e.g. 22:00 - 06:00)
            if (nowMinutes >= startMin || nowMinutes < endMin) return true;
        }
    }
    return false;
}

void ScheduleManager::updateFromJSON(const char* json) {
    JsonDocument doc;
    if (deserializeJson(doc, json) != DeserializationError::Ok) {
        // Serial.println("[SCHED] Failed to parse schedule JSON");
        return;
    }

    _enabled = doc["enabled"] | false;
    JsonArray windows = doc["windows"].as<JsonArray>();
    _windowCount = 0;

    for (JsonObject w : windows) {
        if (_windowCount >= 4) break;

        const char* start = w["start"];  // "06:00"
        const char* end   = w["end"];    // "09:00"

        if (start && end) {
            _windows[_windowCount].startHour   = atoi(start);
            _windows[_windowCount].startMinute = atoi(start + 3);
            _windows[_windowCount].endHour     = atoi(end);
            _windows[_windowCount].endMinute   = atoi(end + 3);
            _windowCount++;
        }
    }

    // Parse timezone if provided
    const char* tz = doc["timezone"] | "";
    if (strlen(tz) > 0) {
        _tzOffsetHours   = atoi(tz);  // e.g. "+05" -> 5
        _tzOffsetMinutes = abs(atoi(tz + 4));  // e.g. ":30" -> 30
        if (tz[0] == '-') _tzOffsetMinutes = -_tzOffsetMinutes;
    }

    saveToNVS();
    // Serial.printf("[SCHED] Updated: %s, %d windows\n",
    //               _enabled ? "enabled" : "disabled", _windowCount);
}

void ScheduleManager::disable() {
    _enabled = false;
    saveToNVS();
    // Serial.println("[SCHED] Schedule disabled — tracking 24/7");
}

bool ScheduleManager::isActive() {
    return _enabled;
}

void ScheduleManager::saveToNVS() {
    schedPrefs.begin("schedule", false);
    schedPrefs.putBool("enabled", _enabled);
    schedPrefs.putUChar("count", _windowCount);
    schedPrefs.putChar("tzH", _tzOffsetHours);
    schedPrefs.putChar("tzM", _tzOffsetMinutes);

    for (uint8_t i = 0; i < _windowCount; i++) {
        String key = "w" + String(i);
        uint32_t packed = (_windows[i].startHour << 24) |
                          (_windows[i].startMinute << 16) |
                          (_windows[i].endHour << 8) |
                          _windows[i].endMinute;
        schedPrefs.putUInt(key.c_str(), packed);
    }

    schedPrefs.end();
}

void ScheduleManager::loadFromNVS() {
    schedPrefs.begin("schedule", false); // false = Read/Write (auto-creates namespace if missing)

    if (schedPrefs.isKey("enabled")) {
        _enabled     = schedPrefs.getBool("enabled", false);
        _windowCount = schedPrefs.getUChar("count", 0);
        _tzOffsetHours   = schedPrefs.getChar("tzH", 5);
        _tzOffsetMinutes = schedPrefs.getChar("tzM", 30);

        for (uint8_t i = 0; i < _windowCount && i < 4; i++) {
            String key = "w" + String(i);
            uint32_t packed = schedPrefs.getUInt(key.c_str(), 0);
            _windows[i].startHour   = (packed >> 24) & 0xFF;
            _windows[i].startMinute = (packed >> 16) & 0xFF;
            _windows[i].endHour     = (packed >> 8)  & 0xFF;
            _windows[i].endMinute   = packed & 0xFF;
        }
    }

    schedPrefs.end();
}
