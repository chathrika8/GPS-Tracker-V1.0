#include "gsm_manager.h"
#include "config.h"

GSMManager gsmManager;

// UART1 instance for SIM800L to escape UART0 collision
static HardwareSerial gsmSerial(1);

void GSMManager::begin() {
    _serial = &gsmSerial;
    _serial->begin(115200, SERIAL_8N1, SIM800L_RX, SIM800L_TX);
    delay(100);

    _modem  = new TinyGsm(*_serial);
    _client = new TinyGsmClient(*_modem);

    Serial.println("[GSM] Initializing modem...");

    // Hardware reset if pin defined
    pinMode(SIM800L_RST, OUTPUT);
    resetModem();

    // Initialize modem
    if (!_modem->restart()) {
        Serial.println("[GSM] Modem restart failed, trying init...");
        _modem->init();
    }

    String modemInfo = _modem->getModemInfo();
    Serial.printf("[GSM] Modem: %s\n", modemInfo.c_str());

    // Network registration and GPRS connection will be handled
    // asynchronously by the Uplink task to prevent WDT resets in setup().
}

bool GSMManager::connectGPRS() {
    Serial.printf("[GSM] Connecting GPRS (APN: %s)...\n", GPRS_APN);
    if (!_modem->gprsConnect(GPRS_APN, GPRS_USER, GPRS_PASS)) {
        Serial.println("[GSM] GPRS connection failed!");
        return false;
    }
    Serial.println("[GSM] GPRS connected.");
    return true;
}

void GSMManager::ensureConnection() {
    if (!_modem->isNetworkConnected()) {
        Serial.println("[GSM] Network lost, re-registering...");
        unsigned long start = millis();
        // Manual wait loop to ensure vTaskDelay feeds the WDT on ESP32-C3
        while (!_modem->isNetworkConnected() && (millis() - start < 30000L)) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    if (!_modem->isGprsConnected()) {
        Serial.println("[GSM] GPRS lost, reconnecting...");
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
    if (rssi == 99) return 0;        // unknown
    return map(rssi, 0, 31, 0, 100);
}

bool GSMManager::isRegistered() {
    return _modem->isNetworkConnected();
}

String GSMManager::getOperator() {
    return _modem->getOperator();
}

TinyGsmClient& GSMManager::getClient() {
    return *_client;
}

TinyGsm& GSMManager::getModem() {
    return *_modem;
}

void GSMManager::resetModem() {
    digitalWrite(SIM800L_RST, LOW);
    delay(150);
    digitalWrite(SIM800L_RST, HIGH);
    delay(3000); // Wait for modem to boot
}

void GSMManager::setAlarm(const char* datetime) {
    // AT+CALA="2026/03/22,17:00:00+22",0,0,"GPS"
    String cmd = "AT+CALA=\"" + String(datetime) + "\",0,0,\"GPS\"";
    _modem->sendAT(cmd.c_str());
    _modem->waitResponse(1000);
    Serial.printf("[GSM] Alarm set: %s\n", datetime);
}

void GSMManager::clearAlarm() {
    _modem->sendAT("+CALD=0");
    _modem->waitResponse(1000);
}

void GSMManager::setFunctionality(int mode) {
    // AT+CFUN=0 (minimum, keeps RTC) or AT+CFUN=1 (full, keeps SMS/RI)
    String cmd = "+CFUN=" + String(mode);
    _modem->sendAT(cmd.c_str());
    _modem->waitResponse(5000);
    Serial.printf("[GSM] CFUN set to %d\n", mode);
}
