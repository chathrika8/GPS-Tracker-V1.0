#pragma once

#include <cstddef>

class OTAManager {
public:
    void begin();
    void checkAndUpdate();    // Check GitHub + Supabase, update if available
    bool isUpdateAvailable();
    const char* getAvailableVersion();

private:
    char _availableVersion[16] = {0};
    bool _updateAvailable = false;

    bool checkGitHub();
    bool checkSupabase();
    bool downloadAndFlash(const char* url, size_t expectedSize, const char* sha256);
};

extern OTAManager otaManager;
