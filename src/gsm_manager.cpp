#include "gsm_manager.h"
#include "log.h"
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

    LOG("[GSM] Initializing modem...");

    pinMode(SIM800L_RST, OUTPUT);
    resetModem();

    if (!_modem->restart()) {
        LOG("[GSM] Modem restart failed, falling back to init()");
        _modem->init();
    }

    LOG("[GSM] Modem: %s\n", _modem->getModemInfo().c_str());

    // Network registration and GPRS are handled by the Uplink task so
    // that a slow SIM attach doesn't block setup() and freeze the display.
}

bool GSMManager::connectGPRS() {
    LOG("[GSM] Connecting GPRS (APN: %s)...\n", GPRS_APN);
    if (!_modem->gprsConnect(GPRS_APN, GPRS_USER, GPRS_PASS)) {
        LOG("[GSM] GPRS connect failed");
        return false;
    }
    LOG("[GSM] GPRS up");
    return true;
}

void GSMManager::ensureConnection() {
    if (!_modem->isNetworkConnected()) {
        LOG("[GSM] Lost network — waiting for re-registration...");
        unsigned long start = millis();
        // Use vTaskDelay instead of delay() so the FreeRTOS watchdog stays fed
        while (!_modem->isNetworkConnected() && (millis() - start < 30000L)) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    if (!_modem->isGprsConnected()) {
        LOG("[GSM] GPRS down — reconnecting...");
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

    // Poll for the modem coming up rather than blocking 3 s unconditionally.
    // SIM800L typically responds in ~1.2 s but worst-case is ~3 s, so cap
    // the wait there. Saves ~1.5–2 s on the typical boot.
    unsigned long deadline = millis() + 3000;
    while (millis() < deadline) {
        if (_modem->testAT(200)) return;
        delay(100);
    }
}

// Cell-tower position via AT+CIPGSMLOC. Requires GPRS to be up because the
// SIM800 firmware reaches out to the cell-tower DB itself over the bearer.
// The TinyGsm getGsmLocation() signature takes lon BEFORE lat (gotcha).
bool GSMManager::getCellLocation(double* lat, double* lon) {
    if (!_modem || !lat || !lon) return false;
    if (!_modem->isGprsConnected()) return false;

    float fLat = 0, fLon = 0, fAcc = 0;
    int   yr = 0, mo = 0, dy = 0, hr = 0, mi = 0, sc = 0;
    if (!_modem->getGsmLocation(&fLon, &fLat, &fAcc,
                                &yr, &mo, &dy, &hr, &mi, &sc)) {
        return false;
    }
    if (fLat == 0.0f && fLon == 0.0f) return false;
    *lat = (double)fLat;
    *lon = (double)fLon;
    return true;
}

// Parse AT+CCLK response into a UTC epoch. Returns 0 if not synchronised.
// CCLK format: "yy/MM/dd,HH:mm:ss±zz" where zz is the offset in quarter-hours.
uint32_t GSMManager::getNetworkUtcEpoch() {
    if (!_modem) return 0;

    int   year, month, day, hour, minute, second;
    float tz;
    if (!_modem->getNetworkTime(&year, &month, &day,
                                &hour, &minute, &second, &tz)) {
        return 0;
    }

    // Convert local time to UTC by subtracting the timezone offset (hours)
    long offsetSec = (long)(tz * 3600.0f);

    // Days from epoch (1970-01-01) to year-month-day
    uint32_t days = 0;
    for (int y = 1970; y < year; y++) {
        bool leap = (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0));
        days += leap ? 366 : 365;
    }
    static const int dpm[] = { 0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
    bool leapYear = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
    for (int m = 1; m < month; m++) {
        days += dpm[m];
        if (m == 2 && leapYear) days++;
    }
    days += day - 1;

    long epoch = (long)days * 86400L
               + (long)hour   * 3600L
               + (long)minute * 60L
               + (long)second
               - offsetSec;
    if (epoch < 0) return 0;
    return (uint32_t)epoch;
}

void GSMManager::setAlarm(const char* datetime) {
    // Format: "2026/03/22,17:00:00+22"  (offset in quarters of an hour)
    String cmd = "AT+CALA=\"" + String(datetime) + "\",0,0,\"GPS\"";
    _modem->sendAT(cmd.c_str());
    _modem->waitResponse(1000);
    LOG("[GSM] RTC alarm set: %s\n", datetime);
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
    LOG("[GSM] CFUN=%d\n", mode);
}
