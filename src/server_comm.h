#pragma once

#include "packet_buffer.h"
#include "gsm_manager.h"

class ServerComm {
public:
    void begin();

    // Send a batch of packets from the buffer via GPRS
    // Returns number of packets successfully sent
    int    sendBatch(int maxCount);
    bool   testConnectivity();

    int    getLastHttpCode() { return _lastHttpCode; }
    String getLastResponse() { return _lastResponse; }
    String getTcpStage()     { return _tcpStage; }
    size_t getTcpHdrSent()   { return _tcpHdrSent; }
    size_t getTcpBodSent()   { return _tcpBodSent; }
    size_t getTcpBodLen()    { return _tcpBodLen; }

private:
    bool   postJSON(const char* host, const char* endpoint, const String& body);
    String buildBatchJSON(GPSPacket* packets, int count);

    int    _lastHttpCode = 0;
    String _lastResponse = "";
    String _tcpStage     = "IDLE";
    size_t _tcpHdrSent   = 0;
    size_t _tcpBodSent   = 0;
    size_t _tcpBodLen    = 0;
};

extern ServerComm serverComm;
