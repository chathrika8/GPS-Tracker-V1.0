#include "gsm_manager.h"
#include "config.h"

GSMManager gsmManager;

// Use UART1 so SIM800L doesn't share UART0 with GPS/debug output
static HardwareSerial gsmSerial(1);

void GSMManager::begin() {
    _serial = &gsmSerial;
    _serial->begin(115200, SERIAL_8N1, SIM800L_RX, SIM800L_TX);
    delay(100);

    _modem  = new TinyGsm(*_serial);
    _client = new TinyGsmClient(*_modem);

    Serial.println("[GSM] Initializing modem...");

    pinMode(SIM800L_RST, OUTPUT);
    resetModem();

    if (!_modem->restart()) {
        Serial.println("[GSM] Modem restart failed, falling back to init()");
        _modem->init();
    }

    Serial.printf("[GSM] Modem: %s\n", _modem->getModemInfo().c_str());

    // Network registration and GPRS are handled by the Uplink task so
    // that a slow SIM attach doesn't block setup() and freeze the display.
}

bool GSMManager::connectGPRS() {
    Serial.printf("[GSM] Connecting GPRS (APN: %s)...\n", GPRS_APN);
    if (!_modem->gprsConnect(GPRS_APN, GPRS_USER, GPRS_PASS)) {
        Serial.println("[GSM] GPRS connect failed");
        return false;
    }
    Serial.println("[GSM] GPRS up");
    return true;
}

void GSMManager::ensureConnection() {
    if (!_modem->isNetworkConnected()) {
        Serial.println("[GSM] Lost network — waiting for re-registration...");
        unsigned long start = millis();
        // Use vTaskDelay instead of delay() so the FreeRTOS watchdog stays fed
        while (!_modem->isNetworkConnected() && (millis() - start < 30000L)) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    if (!_modem->isGprsConnected()) {
        Serial.println("[GSM] GPRS down — reconnecting...");
        connectGPRS();
    }
}

bool GSMManager::isGprsConnected() {
    return _modem->isGprsConnected();
}

int GSMManager::getSignalStrength() {
    return _modem->getSignalQuality();
}

int GSMManager::getSignalPercent() {
    int rssi = getSignalStrength();
    if (rssi == 99) return 0;  // 99 = unknown / not detectable
    return map(rssi, 0, 31, 0, 100);
}

bool GSMManager::isRegistered() {
    return _modem->isNetworkConnected();
}

String GSMManager::getOperator() {
    return _modem->getOperator();
}

TinyGsmClient& GSMManager::getClient() { return *_client; }
TinyGsm&       GSMManager::getModem()  { return *_modem;  }

void GSMManager::resetModem() {
    digitalWrite(SIM800L_RST, LOW);
    delay(150);
    digitalWrite(SIM800L_RST, HIGH);
    delay(3000);  // SIM800L needs ~3 s to boot after PWRKEY pulse
}

void GSMManager::setAlarm(const char* datetime) {
    // Format: "2026/03/22,17:00:00+22"  (offset in quarters of an hour)
    String cmd = "AT+CALA=\"" + String(datetime) + "\",0,0,\"GPS\"";
    _modem->sendAT(cmd.c_str());
    _modem->waitResponse(1000);
    Serial.printf("[GSM] RTC alarm set: %s\n", datetime);
}

void GSMManager::clearAlarm() {
    _modem->sendAT("+CALD=0");
    _modem->waitResponse(1000);
}

void GSMManager::setFunctionality(int mode) {
    // AT+CFUN=0 — minimum power (RTC stays alive, radio off)
    // AT+CFUN=1 — full function (needed for SMS/RI wake)
    String cmd = "+CFUN=" + String(mode);
    _modem->sendAT(cmd.c_str());
    _modem->waitResponse(5000);
    Serial.printf("[GSM] CFUN=%d\n", mode);
}
