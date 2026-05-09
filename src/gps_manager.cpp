#include "gps_manager.h"
#include "config.h"
#include <Preferences.h>

GPSManager gpsManager;
static Preferences gpsPrefs;

// UBX-CFG-RATE: 200 ms measurement period → 5 Hz output
static const uint8_t UBX_CFG_RATE_5HZ[] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
    0xC8, 0x00,  // measRate = 200 ms
    0x01, 0x00,  // navRate  = 1
    0x01, 0x00,  // timeRef  = GPS
    0xDE, 0x6A   // checksum
};

// UBX-CFG-PRT: switch UART1 to 38400 baud, keep UBX+NMEA in/out
// (baudRate little-endian: 0x9600 = 38400.)
static const uint8_t UBX_CFG_PRT_38400[] = {
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00,
    0x01,                          // portID = UART1
    0x00,                          // reserved
    0x00, 0x00,                    // txReady
    0xD0, 0x08, 0x00, 0x00,       // mode: 8N1
    0x00, 0x96, 0x00, 0x00,       // baudRate: 38400
    0x07, 0x00,                    // inProtoMask: UBX+NMEA
    0x03, 0x00,                    // outProtoMask: UBX+NMEA
    0x00, 0x00,                    // flags
    0x00, 0x00,                    // reserved2
    0x93, 0x90                     // checksum (Fletcher-8 over class..payload)
};

// UBX-CFG-MSG: disable sentences we don't parse (saves UART bandwidth)
static const uint8_t UBX_DISABLE_GLL[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
    0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x2A
};
static const uint8_t UBX_DISABLE_GSV[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
    0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x02, 0x38
};
static const uint8_t UBX_DISABLE_GSA[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
    0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x31
};
static const uint8_t UBX_DISABLE_VTG[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
    0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x04, 0x46
};

// UBX-CFG-CFG: save current config to BBR + Flash + EEPROM
static const uint8_t UBX_CFG_SAVE[] = {
    0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00,
    0x00, 0x00, 0x00, 0x00,       // clearMask
    0xFF, 0xFF, 0x00, 0x00,       // saveMask
    0x00, 0x00, 0x00, 0x00,       // loadMask
    0x17,                          // deviceMask: BBR + Flash + EEPROM
    0x31, 0xBF                     // checksum
};

void GPSManager::begin() {
    // Reuse Serial (UART0) so GPS NMEA and debug output share the same port.
    // This works because the NEO-6M outputs NMEA and TinyGPSPlus ignores
    // non-NMEA bytes, so bootloader noise doesn't corrupt parsing.
    _serial = &Serial;
    _serial->begin(GPS_BAUD_DEFAULT, SERIAL_8N1, GPS_RX, GPS_TX);

    configureUBX();
}

void GPSManager::configureUBX() {
    // Strip sentences we don't use to reduce UART load
    sendUBX(UBX_DISABLE_GLL, sizeof(UBX_DISABLE_GLL)); delay(50);
    sendUBX(UBX_DISABLE_GSV, sizeof(UBX_DISABLE_GSV)); delay(50);
    sendUBX(UBX_DISABLE_GSA, sizeof(UBX_DISABLE_GSA)); delay(50);
    sendUBX(UBX_DISABLE_VTG, sizeof(UBX_DISABLE_VTG)); delay(50);

    // Upgrade baud + nav rate together. The 5 Hz path is gated on a non-9600
    // target because at 9600 the UART can't drain the full RMC+GGA pair
    // every 200 ms reliably.
    if (GPS_BAUD_TARGET != GPS_BAUD_DEFAULT) {
        sendUBX(UBX_CFG_PRT_38400, sizeof(UBX_CFG_PRT_38400));
        delay(100);
        _serial->end();
        _serial->begin(GPS_BAUD_TARGET, SERIAL_8N1, GPS_RX, GPS_TX);
        delay(100);

        sendUBX(UBX_CFG_RATE_5HZ, sizeof(UBX_CFG_RATE_5HZ));
        delay(100);
    }

    sendUBX(UBX_CFG_SAVE, sizeof(UBX_CFG_SAVE));
    delay(100);
}

void GPSManager::sendUBX(const uint8_t* msg, size_t len) {
    _serial->write(msg, len);
    // Deliberately no flush() — on ESP32-C3, flush() can block indefinitely
    // if TX and RX wires are swapped (bus contention). The modem's 50–100 ms
    // delay after each command is enough for the bytes to drain.
}

// Fletcher-8 checksum over [startIncl, endExcl); writes 2 bytes at endExcl.
void GPSManager::appendUbxChecksum(uint8_t* buf, size_t startIncl, size_t endExcl) {
    uint8_t ckA = 0, ckB = 0;
    for (size_t i = startIncl; i < endExcl; i++) {
        ckA = (uint8_t)(ckA + buf[i]);
        ckB = (uint8_t)(ckB + ckA);
    }
    buf[endExcl]     = ckA;
    buf[endExcl + 1] = ckB;
}

// ─────────────────────────────────────────────
// AssistNow / cold-start helpers
// ─────────────────────────────────────────────
//
// UBX-AID-INI (class 0x0B, id 0x01) lets us hand the receiver a coarse
// position so it doesn't have to download an almanac before searching for
// satellites. We deliberately omit the time fields here — AssistNow Online
// carries time data in its own MGA-INI-TIME message, and getting the BCD
// date encoding wrong here would actually slow the receiver down.
void GPSManager::injectAidIni(uint32_t /*utcEpoch*/,
                              double lat, double lon, float altM) {
    if (lat == 0.0 && lon == 0.0) return;  // no useful seed

    uint8_t pkt[6 + 48 + 2] = {0};
    pkt[0] = 0xB5; pkt[1] = 0x62;
    pkt[2] = 0x0B; pkt[3] = 0x01;          // AID-INI
    pkt[4] = 0x30; pkt[5] = 0x00;          // length = 48

    int32_t latI = (int32_t)(lat * 1e7);
    int32_t lonI = (int32_t)(lon * 1e7);
    int32_t altC = (int32_t)(altM * 100.0f);   // cm
    uint32_t posAcc = 100000UL;                // 1 km, conservative

    // Payload starts at offset 6
    auto wr32 = [&](size_t o, uint32_t v) {
        pkt[o]     = (uint8_t)(v);
        pkt[o + 1] = (uint8_t)(v >> 8);
        pkt[o + 2] = (uint8_t)(v >> 16);
        pkt[o + 3] = (uint8_t)(v >> 24);
    };

    wr32(6,  (uint32_t)latI);   // ecefXOrLat
    wr32(10, (uint32_t)lonI);   // ecefYOrLon
    wr32(14, (uint32_t)altC);   // ecefZOrAlt
    wr32(18, posAcc);           // posAcc
    // bytes 22..49 stay zero (tmCfg, wnoOrDate, todOrTime, todNs,
    // tAccMs, tAccNs, clkDOrFreq, clkDAccOrFreqAcc)

    // flags @ offset 50: bit 0 (pos) + bit 9 (lla) = 0x00000201
    wr32(50, 0x00000201UL);

    appendUbxChecksum(pkt, 2, 6 + 48);
    sendUBX(pkt, sizeof(pkt));
    delay(50);
}

void GPSManager::injectAssistNowBlob(const uint8_t* data, size_t len) {
    if (!data || len == 0 || !_serial) return;

    // Stream in modest chunks so we don't overrun the UART TX FIFO.
    const size_t CHUNK = 128;
    size_t sent = 0;
    while (sent < len) {
        size_t n = (len - sent > CHUNK) ? CHUNK : (len - sent);
        _serial->write(data + sent, n);
        sent += n;
        delay(10);
    }
}

// ─────────────────────────────────────────────
// Last-fix persistence (NVS)
// ─────────────────────────────────────────────
void GPSManager::saveLastPositionToNVS(double lat, double lon, float altM,
                                       uint32_t epoch) {
    gpsPrefs.begin("gps_last", false);
    gpsPrefs.putDouble("lat", lat);
    gpsPrefs.putDouble("lon", lon);
    gpsPrefs.putFloat ("alt", altM);
    gpsPrefs.putUInt  ("ts",  epoch);
    gpsPrefs.end();
}

bool GPSManager::loadLastPositionFromNVS(double* lat, double* lon,
                                         float* altM, uint32_t* epoch) {
    gpsPrefs.begin("gps_last", true);
    bool have = gpsPrefs.isKey("lat");
    if (have) {
        if (lat)   *lat   = gpsPrefs.getDouble("lat", 0.0);
        if (lon)   *lon   = gpsPrefs.getDouble("lon", 0.0);
        if (altM)  *altM  = gpsPrefs.getFloat ("alt", 0.0f);
        if (epoch) *epoch = gpsPrefs.getUInt  ("ts",  0);
    }
    gpsPrefs.end();
    return have;
}

// ─────────────────────────────────────────────
// UBX parser implementation
// ─────────────────────────────────────────────
void UbxParser::reset() {
    _st = S_S1;
    _cls = _id = 0;
    _len = _idx = 0;
    _ckA = _ckB = _expCkA = 0;
    _ready = false;
}

void UbxParser::onByte(uint8_t b) {
    if (_ready) return;   // wait for consumeFrame()
    switch (_st) {
        case S_S1:
            if (b == 0xB5) { _buf[0] = b; _st = S_S2; }
            break;
        case S_S2:
            if (b == 0x62) { _buf[1] = b; _st = S_CLS; }
            else           { reset(); }
            break;
        case S_CLS:
            _cls = b; _buf[2] = b;
            _ckA = b; _ckB = b;
            _st = S_ID;
            break;
        case S_ID:
            _id = b;  _buf[3] = b;
            _ckA = (uint8_t)(_ckA + b);  _ckB = (uint8_t)(_ckB + _ckA);
            _st = S_LL;
            break;
        case S_LL:
            _len = b; _buf[4] = b;
            _ckA = (uint8_t)(_ckA + b);  _ckB = (uint8_t)(_ckB + _ckA);
            _st = S_LH;
            break;
        case S_LH:
            _len |= ((uint16_t)b << 8);  _buf[5] = b;
            _ckA = (uint8_t)(_ckA + b);  _ckB = (uint8_t)(_ckB + _ckA);
            if (_len > sizeof(_buf) - 8) { reset(); break; }
            _idx = 0;
            _st  = (_len == 0) ? S_KA : S_PL;
            break;
        case S_PL:
            _buf[6 + _idx] = b;
            _ckA = (uint8_t)(_ckA + b);  _ckB = (uint8_t)(_ckB + _ckA);
            if (++_idx >= _len) _st = S_KA;
            break;
        case S_KA:
            _expCkA = b; _buf[6 + _len] = b;
            _st = S_KB;
            break;
        case S_KB:
            _buf[7 + _len] = b;
            if (_expCkA == _ckA && b == _ckB) _ready = true;
            else                              reset();
            break;
    }
}

void UbxParser::consumeFrame(uint8_t* out, size_t* outLen) {
    size_t total = (size_t)8 + _len;
    if (out && outLen) {
        size_t cap  = *outLen;
        size_t copy = (total > cap) ? cap : total;
        memcpy(out, _buf, copy);
        *outLen = total;
    }
    reset();
}

// ─────────────────────────────────────────────
// Reader: feeds bytes to NMEA parser AND UBX parser
// ─────────────────────────────────────────────
void GPSManager::update() {
    while (_serial->available() > 0) {
        uint8_t b = (uint8_t)_serial->read();
        _gps.encode((char)b);
        _ubx.onByte(b);
        if (_ubx.hasFullFrame()) onUbxFrameReady();
    }

    // If a poll opened the capture window > 2 s ago, close it. The replies
    // trickle in over ~1 s at 38400 baud, so 2 s is a safe upper bound.
    if (_capturingEph && (millis() - _ephCaptureStart) > 2000) {
        _capturingEph = false;
    }
}

void GPSManager::onUbxFrameReady() {
    // Only AID-EPH (class 0x0B, id 0x31) with a *full* 104-byte payload is
    // worth caching — the 1-byte form is the receiver telling us "I have no
    // ephemeris for this SV", which is useless to replay.
    bool isFullEph = _capturingEph
                  && _ubx.getCls() == 0x0B
                  && _ubx.getId()  == 0x31
                  && _ubx.getLen() >= 100;
    if (isFullEph) {
        size_t  total  = (size_t)8 + _ubx.getLen();
        if (_ephLen + total <= sizeof(_ephBuf)) {
            size_t lenOut = sizeof(_ephBuf) - _ephLen;
            _ubx.consumeFrame(_ephBuf + _ephLen, &lenOut);
            _ephLen += total;
            _ephCount++;
            return;
        }
    }
    // Discard frame
    uint8_t scratch[120]; size_t s = sizeof(scratch);
    _ubx.consumeFrame(scratch, &s);
}

void GPSManager::pollEphemerides() {
    // UBX-AID-EPH poll-all (no payload) — receiver replies with one frame
    // per SV: full 104 B if it has ephemeris, else 1 B (just SVID).
    static const uint8_t POLL[] = { 0xB5, 0x62, 0x0B, 0x31, 0x00, 0x00, 0x3C, 0xBF };
    _ephLen          = 0;
    _ephCount        = 0;
    _capturingEph    = true;
    _ephCaptureStart = millis();
    sendUBX(POLL, sizeof(POLL));
}

int GPSManager::saveEphemeridesToNVS(uint32_t nowEpoch) {
    _capturingEph = false;
    if (_ephCount == 0 || _ephLen == 0) return 0;

    gpsPrefs.begin("gps_eph", false);
    gpsPrefs.putBytes("blob",  _ephBuf, _ephLen);
    gpsPrefs.putUShort("len",   (uint16_t)_ephLen);
    gpsPrefs.putUChar ("count", _ephCount);
    gpsPrefs.putUInt  ("ts",    nowEpoch);
    gpsPrefs.end();
    return _ephCount;
}

int GPSManager::replayEphemeridesFromNVS(uint32_t nowEpoch, uint32_t maxAgeSec) {
    gpsPrefs.begin("gps_eph", true);
    if (!gpsPrefs.isKey("blob")) { gpsPrefs.end(); return 0; }

    uint32_t saved = gpsPrefs.getUInt("ts", 0);
    if (nowEpoch && saved && (nowEpoch - saved) > maxAgeSec) {
        gpsPrefs.end();
        return 0;   // too stale; the receiver would just discard them anyway
    }
    if (nowEpoch == 0) {
        // No clock yet — replay anyway. Worst case the receiver rejects them
        // and we fall back to standard cold-start behaviour.
    }

    size_t len = gpsPrefs.getUShort("len", 0);
    if (len == 0 || len > sizeof(_ephBuf)) { gpsPrefs.end(); return 0; }
    gpsPrefs.getBytes("blob", _ephBuf, len);
    int count = gpsPrefs.getUChar("count", 0);
    gpsPrefs.end();

    // Each saved frame is a complete UBX packet — sync, header, payload, csum.
    // Walk the buffer and forward each one. The 20 ms sleep gives the
    // receiver time to process before the next frame.
    size_t off = 0;
    int    sent = 0;
    while (off + 8 <= len) {
        if (_ephBuf[off] != 0xB5 || _ephBuf[off + 1] != 0x62) break;
        uint16_t flen = (uint16_t)_ephBuf[off + 4]
                     | ((uint16_t)_ephBuf[off + 5] << 8);
        size_t total = (size_t)8 + flen;
        if (off + total > len) break;
        sendUBX(_ephBuf + off, total);
        off += total;
        sent++;
        delay(20);
    }
    return (sent > 0) ? count : 0;
}

void GPSManager::fillState(DeviceState& state) {
    state.gps_fix = _gps.location.isValid();

    if (state.gps_fix) {
        state.latitude    = _gps.location.lat();
        state.longitude   = _gps.location.lng();
        state.altitude_m  = _gps.altitude.meters();
        state.altitude_ft = _gps.altitude.feet();

        double raw_kmh = _gps.speed.kmph();

        if (raw_kmh < 1.5) {
            // Below noise floor — civilian GPS drifts up to ~1.8 km/h at rest.
            // Clamp to zero rather than showing jitter on the display.
            state.speed_kmh = 0.0;
            state.speed_mph = 0.0;
        } else if (raw_kmh < 10.0) {
            // Walking / slow crawl: smooth heavily to suppress 1–3 km/h jumps
            // between consecutive fixes. 70/30 weighting keeps it stable without
            // feeling too laggy at low speed.
            if (state.speed_kmh == 0.0) {
                state.speed_kmh = raw_kmh;
                state.speed_mph = _gps.speed.mph();
            } else {
                state.speed_kmh = (state.speed_kmh * 0.7) + (raw_kmh * 0.3);
                state.speed_mph = (state.speed_mph * 0.7) + (_gps.speed.mph() * 0.3);
            }
        } else {
            // At driving speed, use the raw value directly — smoothing adds
            // noticeable lag that makes the dashboard feel unresponsive.
            state.speed_kmh = raw_kmh;
            state.speed_mph = _gps.speed.mph();
        }

        state.course = _gps.course.deg();
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

        // Store UTC epoch before applying any timezone offset
        state.utc_epoch = computeUnixEpoch(year, month, day, hour, minute, second);

        // Convert GPS UTC to IST (+05:30) for display only
        minute += 30;
        if (minute >= 60) { minute -= 60; hour++; }
        hour += 5;
        if (hour >= 24) {
            hour -= 24;
            day++;

            int days_in_month[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
            if (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0))
                days_in_month[1] = 29;

            if (day > days_in_month[month - 1]) {
                day = 1;
                if (++month > 12) { month = 1; year++; }
            }
        }

        state.year   = year;
        state.month  = month;
        state.day    = day;
        state.hour   = hour;
        state.minute = minute;
        state.second = second;
    }
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

    return days * 86400UL
         + (uint32_t)hour   * 3600UL
         + (uint32_t)minute * 60UL
         + (uint32_t)second;
}
