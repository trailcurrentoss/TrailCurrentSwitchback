#include <debug.h>
#include "globals.h"
#include "canHelper.h"
#include <OtaUpdate.h>
#include "wifiConfig.h"

// Global credential buffers - writable at runtime
char runtimeSsid[33] = {0};
char runtimePassword[64] = {0};

// Create OTA update handler (3-minute timeout, 180000 ms)
OtaUpdate otaUpdate(180000, runtimeSsid, runtimePassword);

void setup()
{
    Serial.begin(115200);

#if DEBUG == 1
    // Wait for serial monitor connection (up to 3 seconds)
    unsigned long serialWait = millis();
    while (!Serial.available() && (millis() - serialWait < 3000)) {
        delay(10);
    }
#endif

    debugln("\n=== TrailCurrent Switchback ===");
    debugln("CAN-Controlled 6-Channel Relay Module");

    // Initialize WiFi config storage and load stored credentials
    wifiConfig::init();
    wifiConfig::setRuntimeCredentialPtrs(runtimeSsid, runtimePassword);

    if (wifiConfig::loadCredentials(runtimeSsid, runtimePassword)) {
        debugln("[WiFi] Loaded credentials from NVS");
    } else {
        debugln("[WiFi] No credentials in NVS - OTA disabled until provisioned via CAN");
    }

    // Initialize relay pins (all OFF)
    for (int i = 0; i < NUM_RELAYS; i++) {
        pinMode(RELAY_PINS[i], OUTPUT);
        digitalWrite(RELAY_PINS[i], LOW);
    }
    debugln("[Relay] All 6 relays initialized OFF");

    // Initialize buzzer (off)
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    // Initialize OTA
    debugf("[OTA] Device hostname: %s\n", otaUpdate.getHostName().c_str());
    debugln("[OTA] Ready to receive OTA trigger (CAN ID 0x0)");

    // Initialize CAN bus
    canHelper::setupCan();

    debugln("=== Setup Complete ===\n");
}

void loop()
{
    canHelper::canLoop();
}
