#pragma once

#include <cstdint>

class PowerManager {
public:
    void begin();
    void enterDeepSleep(uint64_t wakeAfterUs = 0);
    float readBatteryVoltage(int* percent_out = nullptr);
    uint32_t getUptimeSeconds();

private:
    uint32_t _bootTime = 0;
    void configureWakeSource();
};

extern PowerManager powerManager;
