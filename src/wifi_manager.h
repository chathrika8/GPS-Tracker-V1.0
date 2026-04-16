#pragma once

#include <WiFi.h>
#include "gps_manager.h"

class WiFiManager {
public:
    void begin();
    void enable();
    void disable();
    bool isConnected();
    bool isEnabled();
    int  getRSSI();
    void fillState(DeviceState& state);

private:
    bool _enabled = false;
    void connect();
};

extern WiFiManager wifiManager;
