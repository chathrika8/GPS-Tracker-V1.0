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

    // Network UTC via AT+CCLK. Returns 0 if not yet synced.
    // Used to seed UBX-AID-INI before GPS cold-start.
    uint32_t getNetworkUtcEpoch();

    // Cell-tower geolocation via AT+CIPGSMLOC=1,1. The SIM800 firmware
    // queries Google's cell-tower DB on its own — we don't need any token
    // or Worker route. Accuracy is ~1–5 km, plenty for an AID-INI seed
    // when NVS doesn't have a saved last position (e.g. first boot ever).
    // Returns true on success and writes lat/lon (degrees) into *lat/*lon.
    bool getCellLocation(double* lat, double* lon);

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
