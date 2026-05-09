#pragma once

#include <Arduino.h>

class CommandHandler {
public:
    void begin();
    void pollAndExecute();

private:
    bool   _pendingReboot = false;
    void executeCommand(const char* command, const char* params);
    String httpGet(const char* host, const char* endpoint);
    void   httpPatch(const char* host, const char* endpoint, const String& body);
};

extern CommandHandler commandHandler;
