#pragma once

#include <Arduino.h>

class CommandHandler {
public:
    void begin();
    void pollAndExecute();

private:
    void executeCommand(const char* command, const char* params);
    String httpGet(const char* host, const char* endpoint);
};

extern CommandHandler commandHandler;
