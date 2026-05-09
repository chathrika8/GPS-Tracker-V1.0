#pragma once

#include "packet_buffer.h"
#include "gsm_manager.h"

class ServerComm {
public:
    void begin();

    // Send a batch of packets from the buffer via GPRS.
    // Returns number of packets successfully sent.
    int    sendBatch(int maxCount);
    bool   testConnectivity();

    // Fetch AssistNow MGA blob from the configured Worker route and copy
    // up to *outLen bytes into out. Returns true on HTTP 200 with body.
    bool   fetchAssistNow(uint8_t* out, size_t maxLen, size_t* outLen);

    // Force the keep-alive socket closed (e.g. on long idle, before sleep).
    void   closeSocket();

    int    getLastHttpCode() { return _lastHttpCode; }
    String getLastResponse() { return _lastResponse; }
    String getTcpStage()     { return _tcpStage; }
    size_t getTcpHdrSent()   { return _tcpHdrSent; }
    size_t getTcpBodSent()   { return _tcpBodSent; }
    size_t getTcpBodLen()    { return _tcpBodLen; }

private:
    bool   ensureSocket(const char* host, uint16_t port);
    bool   postBytes(const char* host,
                     const char* endpoint,
                     const char* contentType,
                     const uint8_t* body,
                     size_t bodyLen);

    // Encoders — exactly one is used at runtime, selected by PAYLOAD_PROFILE.
    String      buildBatchJSON(GPSPacket* packets, int count);
    size_t      buildBatchBinary(GPSPacket* packets, int count,
                                 uint8_t* out, size_t maxLen);

    int           _lastHttpCode    = 0;
    String        _lastResponse    = "";
    String        _tcpStage        = "IDLE";
    size_t        _tcpHdrSent      = 0;
    size_t        _tcpBodSent      = 0;
    size_t        _tcpBodLen       = 0;

    // Keep-alive socket bookkeeping
    bool          _socketOpen      = false;
    uint32_t      _socketOpenedAt  = 0;
};

extern ServerComm serverComm;
