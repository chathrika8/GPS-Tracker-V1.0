#pragma once

enum ButtonEvent {
    BTN_NONE,
    BTN_A_SHORT,
    BTN_A_LONG,
    BTN_B_SHORT,
    BTN_B_LONG
};

class ButtonHandler {
public:
    void begin();
    ButtonEvent poll();

private:
    static const unsigned long LONG_PRESS_MS = 1500;

    bool     _btnAState      = true;
    bool     _btnBState      = true;
    bool     _btnBLastState  = true;
    bool     _btnALastState  = true;
    unsigned long _btnAPressTime = 0;
    unsigned long _btnBPressTime = 0;
    unsigned long _lastDebounceA = 0;
    unsigned long _lastDebounceB = 0;
    bool     _btnAHandled    = false;
    bool     _btnBHandled    = false;
};

extern ButtonHandler buttonHandler;
