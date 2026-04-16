#pragma once

#include <Adafruit_SH110X.h>
#include <Adafruit_GFX.h>
#include "gps_manager.h"   // for DeviceState

class DisplayManager {
public:
    void begin();
    void render(const DeviceState& state);
    void nextScreen();
    void toggleDisplay();
    uint8_t getCurrentScreen() { return _currentScreen; }

private:
    Adafruit_SH1107* _display = nullptr;
    uint8_t _currentScreen = 0;
    bool    _displayOn     = true;

    static const uint8_t NUM_SCREENS = 7;

    void renderMainScreen(const DeviceState& state);
    void renderGSMScreen(const DeviceState& state);
    void renderCoordsScreen(const DeviceState& state);
    void renderSystemScreen(const DeviceState& state);
    void renderUplinkScreen(const DeviceState& state);
    void renderTcpDebugScreen(const DeviceState& state);
    void renderModuleDiagsScreen(const DeviceState& state);

    void drawSignalBars(int x, int y, int percent, bool animate_loading = false);
    void drawHeader(const DeviceState& state);
    void drawFooter(const DeviceState& state);
};

extern DisplayManager displayManager;
