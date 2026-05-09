#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#define TINY_GSM_YIELD() vTaskDelay(pdMS_TO_TICKS(1))

// TINY_GSM_MODEM_SIM800 is defined in platformio.ini build_flags
#include <TinyGsmClient.h>
#include <HardwareSerial.h>

class GSMManager {
public:
    void begin();
    void ensureConnection();
    bool isGprsConnected();
    int  getSignalStrength();   // raw 0-31
    int  getSignalPercent();    // mapped 0-100
    bool isRegistered();
    String getOperator();

    // HTTP methods via TinyGSM
    TinyGsmClient& getClient();
    TinyGsm&       getModem();

    // SIM800L RTC alarm for deep sleep wake
    void setAlarm(const char* datetime);  // "2026/03/22,17:00:00+22"
    void clearAlarm();
    void setFunctionality(int mode);      // AT+CFUN=0 or 1

private:
    TinyGsm*             _modem  = nullptr;
    TinyGsmClient*       _client = nullptr;
    HardwareSerial*      _serial      = nullptr;

    bool connectGPRS();
    void resetModem();
};

extern GSMManager gsmManager;
