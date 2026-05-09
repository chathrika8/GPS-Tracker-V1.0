#pragma once

#include <Arduino.h>
#include "gps_manager.h" // For DeviceState

struct SmsEntry {
    int index;
    char sender[20];       // phone number or contact name
    char timestamp[20];    // "26/05/09,16:30:00"
    char body[161];        // SMS body (max 160 chars + null)
    bool unread;
};

class SmsManager {
public:
    void begin();
    void pollSms();
    
    void fillState(DeviceState& state);
    void markRead(int stateIndex);

private:
    static const uint8_t MAX_SMS = 10;
    
    SmsEntry _smsBuffer[MAX_SMS];
    uint8_t _smsCount = 0;
    uint8_t _unreadSmsCount = 0;
};

extern SmsManager smsManager;
