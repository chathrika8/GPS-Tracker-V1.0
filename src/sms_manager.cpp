#include "sms_manager.h"
#include "gsm_manager.h"
#include "log.h"
#include <string.h>
#include <stdlib.h>

SmsManager smsManager;

// ── Private helpers (zero heap allocations) ──────────────────────────────────

// Trim trailing \r, \n, spaces in-place
static void trimBuf(char* buf) {
    int len = (int)strlen(buf);
    while (len > 0 && (buf[len-1] == '\r' || buf[len-1] == '\n' || buf[len-1] == ' ')) {
        buf[--len] = '\0';
    }
}

// Return true if buf looks like a UCS2 hex string (all hex, len%4==0, mostly 00xx pairs)
static bool looksLikeUcs2(const char* buf, size_t len) {
    if (len < 4 || len % 4 != 0) return false;
    int zeroPairs = 0;
    for (size_t i = 0; i < len; i++) {
        char c = buf[i];
        bool isHex = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
        if (!isHex) return false;
        if (i % 4 == 0 && c == '0' && buf[i+1] == '0') zeroPairs++;
    }
    return zeroPairs >= (int)(len / 8);
}

// Decode UCS2 hex string into ASCII in dst, no heap used
static void decodeUcs2(const char* src, size_t srcLen, char* dst, size_t dstLen) {
    size_t out = 0;
    for (size_t i = 0; i + 3 < srcLen && out + 1 < dstLen; i += 4) {
        char hex[3] = { src[i+2], src[i+3], '\0' };
        char c = (char)strtol(hex, NULL, 16);
        dst[out++] = (c >= 32 && c <= 126) ? c : '?';
    }
    dst[out] = '\0';
}

// Extract the Nth quoted token (1-indexed) from line into out, no heap
static bool extractQuoted(const char* line, int n, char* out, size_t outLen) {
    out[0] = '\0';
    int count = 0;
    const char* p = line;
    while (*p) {
        if (*p == '"') {
            count++;
            if (count == (n * 2 - 1)) {
                p++;
                const char* end = strchr(p, '"');
                if (!end) return false;
                size_t len = (size_t)(end - p);
                if (len >= outLen) len = outLen - 1;
                memcpy(out, p, len);
                out[len] = '\0';
                return true;
            }
        }
        p++;
    }
    return false;
}

// Extract the last quoted token from line into out
static bool extractLastQuoted(const char* line, char* out, size_t outLen) {
    out[0] = '\0';
    size_t lineLen = strlen(line);
    if (lineLen < 2) return false;
    const char* closeQ = line + lineLen - 1;
    while (closeQ > line && *closeQ != '"') closeQ--;
    if (*closeQ != '"') return false;
    const char* openQ = closeQ - 1;
    while (openQ > line && *openQ != '"') openQ--;
    if (*openQ != '"' || openQ == closeQ) return false;
    size_t len = (size_t)(closeQ - openQ - 1);
    if (len >= outLen) len = outLen - 1;
    memcpy(out, openQ + 1, len);
    out[len] = '\0';
    return true;
}

// ── SmsManager methods ───────────────────────────────────────────────────────

// Drain leftover bytes from the modem stream so subsequent AT commands
// don't trip on stale data. Bounded so it can never spin.
static void drainStream(TinyGsm& modem, uint32_t budgetMs) {
    unsigned long deadline = millis() + budgetMs;
    while ((long)(millis() - deadline) < 0) {
        if (!modem.stream.available()) return;
        modem.stream.read();
    }
}

void SmsManager::begin() {
    LOG("[SMS] Configuring modem for SMS...");
    TinyGsm& modem = gsmManager.getModem();

    modem.sendAT(GF("+CMGF=1"));         // Text mode
    if (modem.waitResponse(1000) != 1) {
        LOG("[SMS] +CMGF=1 not acknowledged — polling will be skipped");
    }
    modem.sendAT(GF("+CSCS=\"GSM\""));   // GSM character set
    if (modem.waitResponse(1000) != 1) {
        LOG("[SMS] +CSCS not acknowledged");
    }
}

bool SmsManager::pollSms() {
    LOG("[SMS] Polling messages...");
    TinyGsm& modem = gsmManager.getModem();

    // Parse into workBuffer; only commit to _smsBuffer on success so a
    // partial / failed poll never overwrites the cached list.
    static SmsEntry workBuffer[MAX_SMS];
    memset(workBuffer, 0, sizeof(workBuffer));
    uint8_t newSmsCount    = 0;
    uint8_t newUnreadCount = 0;

    int toDelete[20];
    int deleteCount = 0;

    // Static buffers — kept off the uplinkTask stack. A 160-char UCS2 body
    // is 640 hex chars, so the line buffer must be at least that.
    static char lineBuf[700];
    static char bodyBuf[700];

    // Tighten the Stream timeout so readBytesUntil can't spin for a full
    // second per partial line. Restored on every exit path.
    unsigned long prevTimeout = modem.stream.getTimeout();
    modem.stream.setTimeout(50);

    modem.sendAT(GF("+CMGL=\"ALL\""));

    const unsigned long deadline          = millis() + 8000;
    const unsigned long firstByteDeadline = millis() + 2000;
    bool sawAnyByte    = false;
    bool gotTerminator = false;
    bool sawError      = false;

    while ((long)(millis() - deadline) < 0) {
        vTaskDelay(pdMS_TO_TICKS(1));

        if (!modem.stream.available()) {
            // Modem hasn't said anything at all — abort early so we don't
            // burn the full 8 s budget when the radio is unhappy.
            if (!sawAnyByte && (long)(millis() - firstByteDeadline) >= 0) {
                LOG("[SMS] No response from modem, aborting");
                break;
            }
            continue;
        }
        sawAnyByte = true;

        size_t len = modem.stream.readBytesUntil('\n', lineBuf, sizeof(lineBuf) - 1);
        lineBuf[len] = '\0';
        trimBuf(lineBuf);
        if (lineBuf[0] == '\0') continue;

        if (strcmp(lineBuf, "OK") == 0) { gotTerminator = true; break; }
        if (strcmp(lineBuf, "ERROR") == 0
            || strncmp(lineBuf, "+CMS ERROR", 10) == 0
            || strncmp(lineBuf, "+CME ERROR", 10) == 0) {
            LOG("[SMS] Modem returned %s\n", lineBuf);
            sawError = true;
            break;
        }
        if (strncmp(lineBuf, "+CMGL:", 6) != 0) continue;  // skip URCs / blank lines

        // ── Parse SIM index ─────────────────────────────────────────────
        // SIM800 SMS storage indexes are small positive integers; treat
        // anything else as a malformed header and skip it.
        int simIdx = atoi(lineBuf + 7);
        if (simIdx <= 0 || simIdx > 1000) continue;

        // ── Allocate slot in the working buffer ────────────────────────
        SmsEntry* e;
        bool overflow = (newSmsCount >= MAX_SMS);
        if (!overflow) {
            e = &workBuffer[newSmsCount];
        } else {
            if (deleteCount < 20) toDelete[deleteCount++] = workBuffer[0].index;
            if (workBuffer[0].unread && newUnreadCount > 0) newUnreadCount--;
            for (int i = 0; i < MAX_SMS - 1; i++) workBuffer[i] = workBuffer[i + 1];
            e = &workBuffer[MAX_SMS - 1];
        }
        memset(e, 0, sizeof(SmsEntry));
        e->index = simIdx;

        // ── Read/Unread, restoring locally tracked state ────────────────
        e->unread = (strstr(lineBuf, "\"REC UNREAD\"") != NULL);
        for (int i = 0; i < _smsCount; i++) {
            if (_smsBuffer[i].index == simIdx) {
                e->unread = _smsBuffer[i].unread;
                break;
            }
        }

        // ── Sender (2nd quoted token: <oa>/<da>) ──────────────────────────
        // +CMGL: <idx>,"<stat>","<oa>",[<alpha>],"<scts>" — when <alpha>
        // is empty the 3rd token is the timestamp, not the sender.
        extractQuoted(lineBuf, 2, e->sender, sizeof(e->sender));
        extractLastQuoted(lineBuf, e->timestamp, sizeof(e->timestamp));

        // ── Body (next non-empty line) ─────────────────────────────────
        unsigned long bodyDeadline = millis() + 1500;
        bool gotBody = false;
        while ((long)(millis() - bodyDeadline) < 0
               && (long)(millis() - deadline) < 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
            if (!modem.stream.available()) continue;

            size_t bLen = modem.stream.readBytesUntil('\n', bodyBuf, sizeof(bodyBuf) - 1);
            bodyBuf[bLen] = '\0';
            trimBuf(bodyBuf);
            if (bodyBuf[0] == '\0') continue;  // blank separator

            size_t bodyLen = strlen(bodyBuf);
            if (looksLikeUcs2(bodyBuf, bodyLen)) {
                decodeUcs2(bodyBuf, bodyLen, e->body, sizeof(e->body));
            } else {
                strncpy(e->body, bodyBuf, sizeof(e->body) - 1);
                e->body[sizeof(e->body) - 1] = '\0';
            }
            gotBody = true;
            break;
        }

        if (!gotBody) {
            // Drop the half-parsed slot; the next poll will pick the SMS
            // up again. We do NOT abort the whole poll here — one stuck
            // record shouldn't cost us the rest of the listing.
            LOG("[SMS] Body timeout for idx %d, dropping\n", simIdx);
            memset(e, 0, sizeof(SmsEntry));
            if (overflow) {
                // We already shifted the array down. Keep newSmsCount as
                // is so the cleared trailing slot is reused next round.
            }
            continue;
        }

        if (!overflow) newSmsCount++;
        if (e->unread) newUnreadCount++;
    }

    drainStream(modem, 100);
    modem.stream.setTimeout(prevTimeout);

    if (sawError || !gotTerminator) {
        LOG("[SMS] Poll did not complete (term=%d err=%d any=%d) — keeping cached list\n",
            (int)gotTerminator, (int)sawError, (int)sawAnyByte);
        return false;
    }

    // ── Commit ──────────────────────────────────────────────────────────
    memcpy(_smsBuffer, workBuffer, sizeof(_smsBuffer));
    _smsCount       = newSmsCount;
    _unreadSmsCount = newUnreadCount;

    // Best-effort delete of overflow messages on the SIM. Failures here
    // are non-fatal — the next poll will try again.
    for (int i = 0; i < deleteCount; i++) {
        modem.sendAT(GF("+CMGD="), toDelete[i]);
        modem.waitResponse(500);
    }

    return true;
}

void SmsManager::fillState(DeviceState& state) {
    state.sms_count = _smsCount;
    state.sms_unread_count = _unreadSmsCount;

    for (int i = 0; i < _smsCount; i++) {
        int r = _smsCount - 1 - i;  // newest first

        strncpy(state.sms_sender[i],  _smsBuffer[r].sender,    sizeof(state.sms_sender[i]) - 1);
        state.sms_sender[i][sizeof(state.sms_sender[i]) - 1]   = '\0';

        strncpy(state.sms_body[i],    _smsBuffer[r].body,      sizeof(state.sms_body[i]) - 1);
        state.sms_body[i][sizeof(state.sms_body[i]) - 1]       = '\0';

        strncpy(state.sms_time[i],    _smsBuffer[r].timestamp, sizeof(state.sms_time[i]) - 1);
        state.sms_time[i][sizeof(state.sms_time[i]) - 1]       = '\0';

        state.sms_unread[i] = _smsBuffer[r].unread;

        strncpy(state.sms_preview[i], _smsBuffer[r].body, sizeof(state.sms_preview[i]) - 1);
        state.sms_preview[i][sizeof(state.sms_preview[i]) - 1] = '\0';
        if (strlen(_smsBuffer[r].body) >= sizeof(state.sms_preview[i]) - 1) {
            state.sms_preview[i][sizeof(state.sms_preview[i]) - 4] = '.';
            state.sms_preview[i][sizeof(state.sms_preview[i]) - 3] = '.';
            state.sms_preview[i][sizeof(state.sms_preview[i]) - 2] = '.';
            state.sms_preview[i][sizeof(state.sms_preview[i]) - 1] = '\0';
        }
    }
}

void SmsManager::markRead(int stateIndex) {
    if (stateIndex >= 0 && stateIndex < _smsCount) {
        int r = _smsCount - 1 - stateIndex;
        if (_smsBuffer[r].unread) {
            _smsBuffer[r].unread = false;
            if (_unreadSmsCount > 0) _unreadSmsCount--;
        }
    }
}
