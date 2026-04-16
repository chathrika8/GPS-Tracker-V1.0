#include "gps_manager.h"
#include "config.h"

GPSManager gpsManager;

// ── UBX command: Set measurement rate to 200ms (5 Hz) ──
static const uint8_t UBX_CFG_RATE_5HZ[] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
    0xC8, 0x00,  // measRate = 200 ms
    0x01, 0x00,  // navRate  = 1
    0x01, 0x00,  // timeRef  = GPS
    0xDE, 0x6A   // checksum
};

// ── UBX command: Set baud rate to 115200 on UART1 ──
static const uint8_t UBX_CFG_PRT_115200[] = {
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00,
    0x01,                          // portID = UART1
    0x00,                          // reserved
    0x00, 0x00,                    // txReady
    0xD0, 0x08, 0x00, 0x00,       // mode: 8N1
    0x00, 0xC2, 0x01, 0x00,       // baudRate: 115200
    0x07, 0x00,                    // inProtoMask: UBX+NMEA
    0x03, 0x00,                    // outProtoMask: UBX+NMEA
    0x00, 0x00,                    // flags
    0x00, 0x00,                    // reserved2
    0xC0, 0x7E                     // checksum
};

// ── UBX command: Disable GLL sentence ──
static const uint8_t UBX_DISABLE_GLL[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
    0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x2A
};

// ── UBX command: Disable GSV sentence ──
static const uint8_t UBX_DISABLE_GSV[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
    0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x02, 0x38
};

// ── UBX command: Disable GSA sentence ──
static const uint8_t UBX_DISABLE_GSA[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
    0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x31
};

// ── UBX command: Disable VTG sentence ──
static const uint8_t UBX_DISABLE_VTG[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
    0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x04, 0x46
};

// ── UBX command: Save config to flash ──
static const uint8_t UBX_CFG_SAVE[] = {
    0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00,
    0x00, 0x00, 0x00, 0x00,       // clearMask
    0xFF, 0xFF, 0x00, 0x00,       // saveMask
    0x00, 0x00, 0x00, 0x00,       // loadMask
    0x17,                          // deviceMask: BBR + Flash + EEPROM
    0x31, 0xBF                     // checksum
};

void GPSManager::begin() {
    // Inherit the global standard Serial (UART0) for GPS to flawlessly absorb ROM/bootloader logs
    _serial = &Serial;
    _serial->begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    
    // Serial.println("[GPS] Configuring NEO-6M...");
    
    // Disable NMEA messages we don't need (optional, Neo-6M defaults to GGA, GLL, GSA, GSV, RMC, VTG)
    configureUBX();

    // Serial.println("[GPS] Ready — 115200 baud, 5 Hz");
}

void GPSManager::configureUBX() {
    // 1. Set measurement rate to 5 Hz (Only if high-speed UART is enabled)
    if (GPS_BAUD_TARGET > 9600) {
        sendUBX(UBX_CFG_RATE_5HZ, sizeof(UBX_CFG_RATE_5HZ));
        delay(100);
    }

    // 2. Disable unnecessary NMEA sentences
    sendUBX(UBX_DISABLE_GLL, sizeof(UBX_DISABLE_GLL));
    delay(50);
    sendUBX(UBX_DISABLE_GSV, sizeof(UBX_DISABLE_GSV));
    delay(50);
    sendUBX(UBX_DISABLE_GSA, sizeof(UBX_DISABLE_GSA));
    delay(50);
    sendUBX(UBX_DISABLE_VTG, sizeof(UBX_DISABLE_VTG));
    delay(50);

    // 3. Switch baud if target is different
    if (GPS_BAUD_TARGET != GPS_BAUD_DEFAULT) {
        sendUBX(UBX_CFG_PRT_115200, sizeof(UBX_CFG_PRT_115200));
        delay(100);

        // 4. Re-open serial at new baud
        _serial->end();
        _serial->begin(GPS_BAUD_TARGET, SERIAL_8N1, GPS_RX, GPS_TX);
        delay(100);
    }

    // 5. Save config to flash
    sendUBX(UBX_CFG_SAVE, sizeof(UBX_CFG_SAVE));
    delay(100);
}

void GPSManager::sendUBX(const uint8_t* msg, size_t len) {
    _serial->write(msg, len);
    // Removed _serial->flush() as it can block indefinitely on ESP32-C3 
    // if there is a hardware bus collision (e.g. TX wired to TX).
}

void GPSManager::update() {
    while (_serial->available() > 0) {
        _gps.encode(_serial->read());
    }
}

void GPSManager::fillState(DeviceState& state) {
    state.gps_fix = _gps.location.isValid();

    if (state.gps_fix) {
        state.latitude   = _gps.location.lat();
        state.longitude  = _gps.location.lng();
        state.altitude_m = _gps.altitude.meters();
        state.altitude_ft = _gps.altitude.feet();
        
        // Advanced Drift Mitigation & Speed Smoothing Algorithm
        double raw_kmh = _gps.speed.kmph();
        
        if (raw_kmh < 1.5) { 
            // Absolute noise cutoff: Civilian GPS drifts up to 0.5 m/s (1.8 km/h).
            state.speed_kmh = 0.0;
            state.speed_mph = 0.0;
        } 
        else if (raw_kmh < 10.0) {
            // Walking speed / Slow crawl: Exponential Smoothing to stop 1-3 km/h "jumping"
            if (state.speed_kmh == 0.0) {
                state.speed_kmh = raw_kmh;
                state.speed_mph = _gps.speed.mph();
            } else {
                state.speed_kmh = (state.speed_kmh * 0.7) + (raw_kmh * 0.3); // Heavy smoothing (30% new, 70% old)
                state.speed_mph = (state.speed_mph * 0.7) + (_gps.speed.mph() * 0.3);
            }
        } 
        else {
            // High speed driving: Zero smoothing to ensure 1Hz dashboard accuracy without lag
            state.speed_kmh = raw_kmh;
            state.speed_mph = _gps.speed.mph();
        }
        
        state.course     = _gps.course.deg();
        strncpy(state.compass, courseToCompass(state.course), sizeof(state.compass));
    }

    state.satellites = _gps.satellites.value();
    state.hdop       = _gps.hdop.hdop();
    state.gps_chars_processed = _gps.charsProcessed();

    if (_gps.time.isValid() && _gps.date.isValid()) {
        int year   = _gps.date.year();
        int month  = _gps.date.month();
        int day    = _gps.date.day();
        int hour   = _gps.time.hour();
        int minute = _gps.time.minute();
        int second = _gps.time.second();

        // Compute proper Unix epoch from UTC values BEFORE IST conversion
        state.utc_epoch = computeUnixEpoch(year, month, day, hour, minute, second);

        // Convert UTC to IST (+5:30) for display
        minute += 30;
        if (minute >= 60) {
            minute -= 60;
            hour += 1;
        }
        hour += 5;
        if (hour >= 24) {
            hour -= 24;
            day += 1;

            // Handle month overflow
            int days_in_month[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
            if (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)) {
                days_in_month[1] = 29;
            }
            if (day > days_in_month[month - 1]) {
                day = 1;
                month += 1;
                if (month > 12) {
                    month = 1;
                    year += 1;
                }
            }
        }

        state.year   = year;
        state.month  = month;
        state.day    = day;
        state.hour   = hour;
        state.minute = minute;
        state.second = second;
    }

    // Simple epoch approximation (seconds since midnight)
    // NOTE: utc_epoch is now set above from proper GPS UTC values
}

const char* GPSManager::courseToCompass(double course) {
    if (course < 0)    course += 360.0;
    if (course >= 360) course -= 360.0;

    static const char* dirs[] = {
        "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
        "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"
    };
    int idx = (int)((course + 11.25) / 22.5) % 16;
    return dirs[idx];
}

uint32_t GPSManager::computeUnixEpoch(int year, int month, int day,
                                       int hour, int minute, int second) {
    // Days from 1970-01-01 to start of given year
    uint32_t days = 0;
    for (int y = 1970; y < year; y++) {
        bool leap = (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0));
        days += leap ? 366 : 365;
    }
    // Days from start of year to start of given month
    static const int dpm[] = { 0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
    bool leapYear = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
    for (int m = 1; m < month; m++) {
        days += dpm[m];
        if (m == 2 && leapYear) days += 1;
    }
    days += day - 1;
    return days * 86400UL + (uint32_t)hour * 3600UL + (uint32_t)minute * 60UL + (uint32_t)second;
}
