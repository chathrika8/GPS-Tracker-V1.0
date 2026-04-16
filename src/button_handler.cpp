#include "button_handler.h"
#include "config.h"
#include <Arduino.h>

ButtonHandler buttonHandler;

void ButtonHandler::begin() {
    pinMode(BTN_A, INPUT_PULLUP);
    pinMode(BTN_B, INPUT_PULLUP);
    // Serial.println("[BTN] Buttons ready (A=GPIO" + String(BTN_A) +
    //                ", B=GPIO" + String(BTN_B) + ")");
}

ButtonEvent ButtonHandler::poll() {
    unsigned long now = millis();
    ButtonEvent event = BTN_NONE;

    // ── Button A ──
    bool readA = digitalRead(BTN_A);
    if (readA != _btnALastState) _lastDebounceA = now;

    if ((now - _lastDebounceA) > BUTTON_DEBOUNCE) {
        if (readA != _btnAState) {
            _btnAState = readA;

            if (_btnAState == LOW) {
                // Pressed
                _btnAPressTime = now;
                _btnAHandled = false;
            } else {
                // Released
                if (!_btnAHandled) {
                    unsigned long duration = now - _btnAPressTime;
                    event = (duration >= LONG_PRESS_MS) ? BTN_A_LONG : BTN_A_SHORT;
                }
            }
        }

        // Detect long press while still held
        if (_btnAState == LOW && !_btnAHandled &&
            (now - _btnAPressTime) >= LONG_PRESS_MS) {
            event = BTN_A_LONG;
            _btnAHandled = true;
        }
    }
    _btnALastState = readA;

    // ── Button B ──
    bool readB = digitalRead(BTN_B);
    if (readB != _btnBLastState) _lastDebounceB = now;

    if ((now - _lastDebounceB) > BUTTON_DEBOUNCE) {
        if (readB != _btnBState) {
            _btnBState = readB;

            if (_btnBState == LOW) {
                _btnBPressTime = now;
                _btnBHandled = false;
            } else {
                if (!_btnBHandled) {
                    unsigned long duration = now - _btnBPressTime;
                    event = (duration >= LONG_PRESS_MS) ? BTN_B_LONG : BTN_B_SHORT;
                }
            }
        }

        if (_btnBState == LOW && !_btnBHandled &&
            (now - _btnBPressTime) >= LONG_PRESS_MS) {
            event = BTN_B_LONG;
            _btnBHandled = true;
        }
    }
    _btnBLastState = readB;

    return event;
}
