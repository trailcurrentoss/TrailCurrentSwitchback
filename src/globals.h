#pragma once
#include <Arduino.h>
#include <debug.h>

// Waveshare ESP32-S3-Relay-6CH pin assignments
#define RELAY01_PIN 1
#define RELAY02_PIN 2
#define RELAY03_PIN 41
#define RELAY04_PIN 42
#define RELAY05_PIN 45
#define RELAY06_PIN 46

#define NUM_RELAYS 6

#define BUZZER_PIN 21
#define RGB_LED_PIN 38

static const int RELAY_PINS[NUM_RELAYS] = {
    RELAY01_PIN, RELAY02_PIN, RELAY03_PIN,
    RELAY04_PIN, RELAY05_PIN, RELAY06_PIN
};

// Current on/off state of each relay (true = ON, false = OFF)
bool relayStates[NUM_RELAYS] = {false, false, false, false, false, false};
