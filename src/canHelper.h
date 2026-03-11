#pragma once
#include "globals.h"
#include <TwaiTaskBased.h>
#include <OtaUpdate.h>
#include "wifiConfig.h"

// Forward declare otaUpdate (defined in main.cpp)
extern OtaUpdate otaUpdate;

#define CAN_TX GPIO_NUM_4
#define CAN_RX GPIO_NUM_5
#define CAN_ID_TOGGLE 0x25
#define CAN_ID_STATUS 0x28

namespace canHelper
{
    /**
     * Send relay status as a bitmask in a single byte
     * Reads actual GPIO pin states via digitalRead()
     * Bit 0 = CH1 ... Bit 5 = CH6 (1 = ON, 0 = OFF)
     */
    void send_status_now()
    {
        uint8_t statusByte = 0;
        for (int i = 0; i < NUM_RELAYS; i++) {
            if (digitalRead(RELAY_PINS[i])) {
                statusByte |= (1 << i);
            }
        }

        twai_message_t message;
        message.identifier = CAN_ID_STATUS;
        message.extd = false;
        message.rtr = false;
        message.data_length_code = 1;
        message.data[0] = statusByte;

        TwaiTaskBased::send(message, pdMS_TO_TICKS(10));
    }

    /**
     * Handle OTA trigger from CAN message ID 0x0
     * Format: 3 bytes [MAC byte 3, MAC byte 4, MAC byte 5]
     */
    void handleOtaTrigger(const uint8_t *data) {
        char updateForHostName[14];
        String currentHostName = otaUpdate.getHostName();

        sprintf(updateForHostName, "esp32-%02X%02X%02X",
                data[0], data[1], data[2]);

        debugf("[OTA] Target hostname: %s\n", updateForHostName);
        debugf("[OTA] Current hostname: %s\n", currentHostName.c_str());

        if (currentHostName.equals(updateForHostName)) {
            debugln("[OTA] Hostname matched - entering OTA mode");
            otaUpdate.waitForOta();
            debugln("[OTA] OTA mode exited - resuming normal operation");
        } else {
            debugln("[OTA] Hostname mismatch - ignoring OTA trigger");
        }
    }

    /**
     * Toggle a single relay by index (0-5)
     */
    void toggleRelay(uint8_t channel) {
        if (channel >= NUM_RELAYS) return;

        relayStates[channel] = !relayStates[channel];
        digitalWrite(RELAY_PINS[channel], relayStates[channel] ? HIGH : LOW);
        debugf("[Relay] CH%d (GPIO %d) %s\n",
               channel + 1, RELAY_PINS[channel],
               relayStates[channel] ? "ON" : "OFF");
    }

    /**
     * Set all relays to a given state
     */
    void setAllRelays(bool state) {
        for (int i = 0; i < NUM_RELAYS; i++) {
            relayStates[i] = state;
            digitalWrite(RELAY_PINS[i], state ? HIGH : LOW);
        }
        debugf("[Relay] All relays %s\n", state ? "ON" : "OFF");
    }

    static void handle_rx_message(const twai_message_t &message)
    {
        if (message.rtr) return;

        // OTA trigger (ID 0x00)
        if (message.identifier == 0x00 && message.data_length_code >= 3) {
            debugln("[OTA] OTA trigger received");
            handleOtaTrigger(message.data);
        }
        // WiFi config (ID 0x01)
        else if (message.identifier == 0x01 && message.data_length_code >= 1) {
            wifiConfig::handleCanMessage(message.data, message.data_length_code);
        }
        // Relay toggle command (ID 0x25)
        else if (message.identifier == CAN_ID_TOGGLE)
        {
            if (message.data[0] <= 5) {
                toggleRelay(message.data[0]);
            }
            else if (message.data[0] == 6 && message.data_length_code >= 2) {
                setAllRelays(message.data[1] != 0);
            }
        }
    }

    static void handle_tx_result(bool success) {
        if (!success) {
            debugln("[CAN] TX FAILED");
        }
    }

    void setupCan()
    {
        if (TwaiTaskBased::begin(CAN_TX, CAN_RX, 500000, TWAI_MODE_NO_ACK)) {
            debugln("[CAN] Driver initialized");
        } else {
            debugln("[CAN] Failed to initialize driver");
            return;
        }

        TwaiTaskBased::onReceive(handle_rx_message);
        TwaiTaskBased::onTransmit(handle_tx_result);
        debugln("[CAN] RX/TX callbacks registered");
    }

    void canLoop()
    {
        wifiConfig::checkTimeout();

        static unsigned long lastStatusTx = 0;
        unsigned long now = millis();
        if (now - lastStatusTx >= 33) {
            lastStatusTx = now;
            send_status_now();
        }
    }
}
