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

void SmsManager::begin() {
    LOG("[SMS] Configuring modem for SMS...");
    TinyGsm& modem = gsmManager.getModem();
    modem.sendAT(GF("+CMGF=1"));         // Text mode
    modem.waitResponse();
    modem.sendAT(GF("+CSCS=\"GSM\""));   // GSM character set
    modem.waitResponse();
}

void SmsManager::pollSms() {
    LOG("[SMS] Polling messages...");
    TinyGsm& modem = gsmManager.getModem();

    // Snapshot old buffer to restore unread states after re-poll
    // Static so it never goes on the task stack
    static SmsEntry oldBuffer[MAX_SMS];
    int oldSmsCount = _smsCount;
    memcpy(oldBuffer, _smsBuffer, sizeof(_smsBuffer));

    _smsCount = 0;
    _unreadSmsCount = 0;
    memset(_smsBuffer, 0, sizeof(_smsBuffer));

    int toDelete[20];
    int deleteCount = 0;

    // Use static buffers so they are never on the heap
    static char lineBuf[256];
    static char bodyBuf[256];

    modem.sendAT(GF("+CMGL=\"ALL\""));

    unsigned long timeout = millis() + 10000;
    while (millis() < timeout) {
        if (!modem.stream.available()) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        size_t len = modem.stream.readBytesUntil('\n', lineBuf, sizeof(lineBuf) - 1);
        lineBuf[len] = '\0';
        trimBuf(lineBuf);

        if (strcmp(lineBuf, "OK") == 0 || strcmp(lineBuf, "ERROR") == 0) break;
        if (strncmp(lineBuf, "+CMGL:", 6) != 0) continue;

        // ── Parse SIM index ──────────────────────────────────────────────────
        int simIdx = atoi(lineBuf + 7);

        // ── Allocate slot ────────────────────────────────────────────────────
        SmsEntry* e;
        if (_smsCount < MAX_SMS) {
            e = &_smsBuffer[_smsCount++];
        } else {
            if (deleteCount < 20) toDelete[deleteCount++] = _smsBuffer[0].index;
            for (int i = 0; i < MAX_SMS - 1; i++) _smsBuffer[i] = _smsBuffer[i + 1];
            e = &_smsBuffer[MAX_SMS - 1];
        }

        memset(e, 0, sizeof(SmsEntry));
        e->index = simIdx;

        // ── Read/Unread ──────────────────────────────────────────────────────
        e->unread = (strstr(lineBuf, "\"REC UNREAD\"") != NULL);

        // Restore locally-tracked state (prevents flicker when modem marks as read)
        for (int i = 0; i < oldSmsCount; i++) {
            if (oldBuffer[i].index == simIdx) {
                e->unread = oldBuffer[i].unread;
                break;
            }
        }
        if (e->unread) _unreadSmsCount++;

        // ── Sender (3rd quoted token) ─────────────────────────────────────────
        extractQuoted(lineBuf, 3, e->sender, sizeof(e->sender));

        // ── Timestamp (last quoted token) ─────────────────────────────────────
        extractLastQuoted(lineBuf, e->timestamp, sizeof(e->timestamp));

        // ── Body (next line) ──────────────────────────────────────────────────
        unsigned long bodyTimeout = millis() + 1000;
        bool gotBody = false;
        while (millis() < bodyTimeout && !gotBody) {
            if (!modem.stream.available()) {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            size_t bLen = modem.stream.readBytesUntil('\n', bodyBuf, sizeof(bodyBuf) - 1);
            bodyBuf[bLen] = '\0';
            trimBuf(bodyBuf);
            gotBody = true;

            size_t bodyLen = strlen(bodyBuf);
            if (looksLikeUcs2(bodyBuf, bodyLen)) {
                decodeUcs2(bodyBuf, bodyLen, e->body, sizeof(e->body));
            } else {
                strncpy(e->body, bodyBuf, sizeof(e->body) - 1);
                e->body[sizeof(e->body) - 1] = '\0';
            }
        }
    }

    // Delete overflow messages from the SIM card
    for (int i = 0; i < deleteCount; i++) {
        modem.sendAT(GF("+CMGD="), toDelete[i]);
        modem.waitResponse(500);
    }
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
