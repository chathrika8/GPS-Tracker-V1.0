#pragma once

#include "gps_manager.h"  // for DeviceState

struct TrackingWindow {
    uint8_t startHour;
    uint8_t startMinute;
    uint8_t endHour;
    uint8_t endMinute;
};

class ScheduleManager {
public:
    void begin();
    void checkWindow(DeviceState& state);
    void updateFromJSON(const char* json);
    void disable();
    bool isActive();

private:
    bool _enabled = false;
    TrackingWindow _windows[4];  // max 4 windows
    uint8_t _windowCount = 0;
    int8_t _tzOffsetHours = 5;   // +05:30 default
    int8_t _tzOffsetMinutes = 30;

    bool isInWindow(uint8_t hour, uint8_t minute);
    void saveToNVS();
    void loadFromNVS();
};

extern ScheduleManager scheduleManager;
