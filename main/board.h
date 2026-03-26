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
#define CAN_ID_TOGGLE           0x25
#define CAN_ID_STATUS           0x28

// CAN baud rate
#define CAN_BAUD_RATE           500  // kbps
