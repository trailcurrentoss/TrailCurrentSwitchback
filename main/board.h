#pragma once

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/twai.h"

// --- Waveshare ESP32-S3-ETH-8DI-8RO-C pin assignments ---

// I2C bus (TCA9554PWR relay expander + PCF85063ATL RTC)
#define BOARD_I2C_SDA_PIN       GPIO_NUM_42
#define BOARD_I2C_SCL_PIN       GPIO_NUM_41
#define BOARD_I2C_FREQ_HZ       100000

// TCA9554PWR I/O expander (8 relay outputs)
#define TCA9554_ADDR            0x20
#define TCA9554_REG_INPUT       0x00
#define TCA9554_REG_OUTPUT      0x01
#define TCA9554_REG_POLARITY    0x02
#define TCA9554_REG_CONFIG      0x03

#define NUM_RELAYS              8

// Digital inputs (optocoupler isolated, active-low)
#define DI1_PIN                 GPIO_NUM_4
#define DI2_PIN                 GPIO_NUM_5
#define DI3_PIN                 GPIO_NUM_6
#define DI4_PIN                 GPIO_NUM_7
#define DI5_PIN                 GPIO_NUM_8
#define DI6_PIN                 GPIO_NUM_9
#define DI7_PIN                 GPIO_NUM_10
#define DI8_PIN                 GPIO_NUM_11

#define NUM_DIN                 8

static const gpio_num_t DIN_PINS[NUM_DIN] = {
    DI1_PIN, DI2_PIN, DI3_PIN, DI4_PIN,
    DI5_PIN, DI6_PIN, DI7_PIN, DI8_PIN
};

// CAN bus (TWAI) — shared with RS485, we use CAN mode
#define CAN_TX_PIN              GPIO_NUM_17
#define CAN_RX_PIN              GPIO_NUM_18

// Buzzer (PWM)
#define BUZZER_PIN              GPIO_NUM_46

// WS2812 RGB LED
#define RGB_LED_PIN             GPIO_NUM_38

// Ethernet (W5500 via SPI)
#define ETH_MOSI_PIN            GPIO_NUM_13
#define ETH_MISO_PIN            GPIO_NUM_14
#define ETH_SCLK_PIN            GPIO_NUM_15
#define ETH_CS_PIN              GPIO_NUM_16
#define ETH_INT_PIN             GPIO_NUM_12
#define ETH_RST_PIN             GPIO_NUM_39

// CAN protocol IDs
#define CAN_ID_OTA              0x00
#define CAN_ID_WIFI_CONFIG      0x01
#define CAN_ID_DISCOVERY_TRIGGER 0x02
#define CAN_ID_TOGGLE_BASE      0x25   // + SWITCHBACK_ADDRESS → 0x25-0x27
#define CAN_ID_STATUS_BASE      0x28   // + SWITCHBACK_ADDRESS → 0x28-0x2A
// Reed-switch / digital-input broadcast — Picket-format frame, 5 Hz.
// Lives in the Picket address pool 0x0A-0x14 (Picket fills 0x0A-0x11; Switchback
// extends with addresses 8-10 at 0x12-0x14). Identical 2-byte wire layout, so any
// PicketStatus consumer decodes Switchback DIs without code changes.
#define CAN_ID_INPUT_BASE       0x12   // + SWITCHBACK_ADDRESS → 0x12-0x14

// CAN baud rate
#define CAN_BAUD_RATE           500  // kbps

// --- Aftline variant (SWITCHBACK_VARIANT=aftline) ---
// Fixed DI-to-trailer-signal mapping. DI6-DI8 unused in aftline mode.
// Trailer wire carrying 12V → opto conducts → GPIO LOW → signal_asserted = true.
#ifdef SWITCHBACK_VARIANT_AFTLINE
#define AFTLINE_DI_CONNECTED       DI1_PIN
#define AFTLINE_DI_LEFT_TURN       DI2_PIN
#define AFTLINE_DI_RIGHT_TURN      DI3_PIN
#define AFTLINE_DI_RUNNING_LIGHTS  DI4_PIN
#define AFTLINE_DI_BRAKES          DI5_PIN

// ADC input for TrailerVoltageMv, fed from a custom voltage-divider PCB on
// the internal header. Must be an ADC-capable GPIO (ADC1: 1-10, ADC2: 11-20)
// not already used by DI1-8, Ethernet, CAN, I2C, buzzer, or RGB LED.
#if !defined(AFTLINE_ADC_GPIO) || !defined(AFTLINE_ADC_CHANNEL) || !defined(AFTLINE_ADC_UNIT)
#error "SWITCHBACK_VARIANT=aftline requires AFTLINE_ADC_GPIO, AFTLINE_ADC_CHANNEL, and AFTLINE_ADC_UNIT to be defined here in board.h. Check the Waveshare ESP32-S3-ETH-8DI-8RO-C internal header pinout, pick an unused ADC-capable GPIO, and set all three. Also set AFTLINE_ADC_DIVIDER_RATIO to match the custom PCB."
#endif

// Voltage divider ratio (Vin / Vout). ADC reads mV at the pin; multiply by
// this ratio to recover the trailer voltage. Example: 5:1 divider → 5.
#ifndef AFTLINE_ADC_DIVIDER_RATIO
#define AFTLINE_ADC_DIVIDER_RATIO  5
#endif
#endif  // SWITCHBACK_VARIANT_AFTLINE
