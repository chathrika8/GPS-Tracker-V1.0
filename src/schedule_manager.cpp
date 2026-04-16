#include "schedule_manager.h"
#include "config.h"
#include <Preferences.h>
#include <ArduinoJson.h>

ScheduleManager scheduleManager;
static Preferences schedPrefs;

void ScheduleManager::begin() {
    _enabled = SCHEDULE_ENABLED;
    loadFromNVS();
}

void ScheduleManager::checkWindow(DeviceState& state) {
    if (!_enabled) {
        state.schedule_active = true;  // no schedule = always on
        return;
    }

    // Convert GPS UTC to local time before window comparison
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
        Serial.printf("[SCHED] Window %s (local %02d:%02d)\n",
                      active ? "entered" : "exited", localHour, localMinute);
        lastActive = active;
    }
}

bool ScheduleManager::isInWindow(uint8_t hour, uint8_t minute) {
    int now = hour * 60 + minute;

    for (uint8_t i = 0; i < _windowCount; i++) {
        int start = _windows[i].startHour * 60 + _windows[i].startMinute;
        int end   = _windows[i].endHour   * 60 + _windows[i].endMinute;

        if (end > start) {
            // Normal window, e.g. 06:00–22:00
            if (now >= start && now < end) return true;
        } else {
            // Overnight window, e.g. 22:00–06:00
            if (now >= start || now < end) return true;
        }
    }
    return false;
}

void ScheduleManager::updateFromJSON(const char* json) {
    JsonDocument doc;
    if (deserializeJson(doc, json) != DeserializationError::Ok) {
        Serial.println("[SCHED] Bad JSON — ignoring update");
        return;
    }

    _enabled     = doc["enabled"] | false;
    _windowCount = 0;

    for (JsonObject w : doc["windows"].as<JsonArray>()) {
        if (_windowCount >= 4) break;

        const char* start = w["start"];  // "HH:MM"
        const char* end   = w["end"];

        if (!start || !end) continue;

        _windows[_windowCount].startHour   = atoi(start);
        _windows[_windowCount].startMinute = atoi(start + 3);
        _windows[_windowCount].endHour     = atoi(end);
        _windows[_windowCount].endMinute   = atoi(end + 3);
        _windowCount++;
    }

    // Timezone string format: "+05:30" or "-04:00"
    const char* tz = doc["timezone"] | "";
    if (strlen(tz) > 0) {
        _tzOffsetHours   = atoi(tz);
        _tzOffsetMinutes = abs(atoi(tz + 4));
        if (tz[0] == '-') _tzOffsetMinutes = -_tzOffsetMinutes;
    }

    saveToNVS();
    Serial.printf("[SCHED] Updated: %s, %d window(s)\n",
                  _enabled ? "enabled" : "disabled", _windowCount);
}

void ScheduleManager::disable() {
    _enabled = false;
    saveToNVS();
}

bool ScheduleManager::isActive() {
    return _enabled;
}

void ScheduleManager::saveToNVS() {
    schedPrefs.begin("schedule", false);
    schedPrefs.putBool("enabled", _enabled);
    schedPrefs.putUChar("count",  _windowCount);
    schedPrefs.putChar("tzH",    _tzOffsetHours);
    schedPrefs.putChar("tzM",    _tzOffsetMinutes);

    for (uint8_t i = 0; i < _windowCount; i++) {
        // Pack four uint8_t fields into one uint32_t to save NVS keys
        uint32_t packed = ((uint32_t)_windows[i].startHour   << 24)
                        | ((uint32_t)_windows[i].startMinute << 16)
                        | ((uint32_t)_windows[i].endHour     <<  8)
                        |            _windows[i].endMinute;
        schedPrefs.putUInt(("w" + String(i)).c_str(), packed);
    }

    schedPrefs.end();
}

void ScheduleManager::loadFromNVS() {
    // Read/Write mode — Preferences auto-creates the namespace on first boot
    schedPrefs.begin("schedule", false);

    if (schedPrefs.isKey("enabled")) {
        _enabled         = schedPrefs.getBool("enabled", false);
        _windowCount     = schedPrefs.getUChar("count", 0);
        _tzOffsetHours   = schedPrefs.getChar("tzH", 5);
        _tzOffsetMinutes = schedPrefs.getChar("tzM", 30);

        for (uint8_t i = 0; i < _windowCount && i < 4; i++) {
            uint32_t packed = schedPrefs.getUInt(("w" + String(i)).c_str(), 0);
            _windows[i].startHour   = (packed >> 24) & 0xFF;
            _windows[i].startMinute = (packed >> 16) & 0xFF;
            _windows[i].endHour     = (packed >>  8) & 0xFF;
            _windows[i].endMinute   =  packed         & 0xFF;
        }
    }

    schedPrefs.end();
}
