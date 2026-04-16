#include "power_manager.h"
#include "gsm_manager.h"
#include "config.h"
#include <Arduino.h>
#include <esp_sleep.h>

PowerManager powerManager;

void PowerManager::begin() {
    _bootTime = millis() / 1000;

    // Check wake reason
    esp_sleep_wakeup_cause_t reason = esp_sleep_get_wakeup_cause();
    switch (reason) {
        case ESP_SLEEP_WAKEUP_GPIO:
            Serial.println("[PWR] Woke from GPIO (SIM800L RI or Button)");
            break;
        case ESP_SLEEP_WAKEUP_TIMER:
            Serial.println("[PWR] Woke from RTC timer");
            break;
        default:
            Serial.println("[PWR] Normal boot");
            break;
    }

    Serial.println("[PWR] Power manager ready.");
}

void PowerManager::enterDeepSleep(uint64_t wakeAfterUs) {
    Serial.println("[PWR] Preparing for deep sleep...");

    // Configure wake sources:
    // 1. Button A (GPIO3) — physical wake
    // 2. SIM800L RI (GPIO9) — RTC alarm or SMS wake
    esp_deep_sleep_enable_gpio_wakeup(
        (1ULL << BTN_A) | (1ULL << SIM800L_RI),
        ESP_GPIO_WAKEUP_GPIO_LOW
    );

    // Optional: timed wake
    if (wakeAfterUs > 0) {
        esp_sleep_enable_timer_wakeup(wakeAfterUs);
        Serial.printf("[PWR] Timer wake set: %llu us\n", wakeAfterUs);
    }

    Serial.println("[PWR] Entering deep sleep...");
    Serial.flush();
    delay(100);

    esp_deep_sleep_start();
    // Does not return
}

float PowerManager::readBatteryVoltage(int* percent_out) {
    // Use SIM800L AT+CBC to read voltage at its VCC pin
    // Response format: +CBC: <bcs>,<bcl>,<voltage>
    //   bcs: 0=not charging, 1=charging, 2=done
    //   bcl: battery charge level 0-100%
    //   voltage: in mV (e.g. 3870 = 3.87V)

    TinyGsm& modem = gsmManager.getModem();
    modem.sendAT("+CBC");

    String response = "";
    if (modem.waitResponse(1000, response) == 1) {
        // Parse "+CBC: 0,85,3870"
        int firstComma = response.indexOf(',');
        int lastComma  = response.lastIndexOf(',');
        if (firstComma > 0 && lastComma > firstComma) {
            // Extract bcl (percentage between first and last comma)
            if (percent_out) {
                String pctStr = response.substring(firstComma + 1, lastComma);
                pctStr.trim();
                *percent_out = pctStr.toInt();
            }
            // Extract voltage (after last comma)
            String mvStr = response.substring(lastComma + 1);
            mvStr.trim();
            int mv = mvStr.toInt();
            if (mv > 0) {
                return mv / 1000.0;
            }
        }
    }

    return 0.0;  // Failed to read
}

uint32_t PowerManager::getUptimeSeconds() {
    return (millis() / 1000) - _bootTime;
}
