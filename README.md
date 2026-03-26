# TrailCurrent Switchback

CAN-controlled 8-channel relay module for the TrailCurrent ecosystem, built on the [Waveshare ESP32-S3-ETH-8DI-8RO-C](https://www.waveshare.com/wiki/ESP32-S3-ETH-8DI-8RO-C).

## Hardware

The Waveshare ESP32-S3-ETH-8DI-8RO-C provides:

- **ESP32-S3** dual-core LX7 @ 240 MHz with WiFi and BLE
- **8 relay channels** (1NO/1NC each), rated 10A @ 250V AC / 10A @ 30V DC, via TCA9554PWR I2C I/O expander
- **8 digital inputs** (optocoupler isolated)
- **CAN bus** with onboard transceiver and 120 ohm termination jumper
- **Ethernet** (W5500 via SPI, 10/100 Mbps)
- **7-36V DC** input (or 5V via USB-C)
- **RTC** (PCF85063ATL) with battery backup header
- **SD card** slot (MMC 1-bit mode)
- DIN-rail mount enclosure

## Firmware

### Pin Assignments

Relays are controlled via TCA9554PWR I2C I/O expander (address 0x20) on I2C SDA=GPIO42, SCL=GPIO41.

| Function   | Pin / Address |
|------------|---------------|
| Relay 1-8  | TCA9554 P0-P7 (I2C 0x20) |
| DI 1-8     | GPIO 4-11     |
| CAN TX     | GPIO 17       |
| CAN RX     | GPIO 18       |
| Buzzer     | GPIO 46       |
| RGB LED    | GPIO 38       |
| ETH MOSI   | GPIO 13       |
| ETH MISO   | GPIO 14       |
| ETH SCLK   | GPIO 15       |
| ETH CS     | GPIO 16       |

### CAN Protocol

CAN bus runs at **500 kbps** in no-ACK mode.

| CAN ID | Direction | Description |
|--------|-----------|-------------|
| `0x00` | RX | **OTA trigger** — 3-byte payload (MAC bytes 3-5) targets a specific device |
| `0x01` | RX | **WiFi config** — multi-message chunked protocol to provision SSID and password |
| `0x25` | RX | **Relay toggle** — toggle a single relay (channel 0-7) or set all relays (channel >= 8, byte 2 = on/off) |
| `0x28` | TX | **Status broadcast** — 1-byte bitmask (bits 0-7 = relay states), sent at ~30 Hz |

### WiFi Provisioning over CAN

WiFi credentials can be sent to the device over the CAN bus using a multi-message protocol on CAN ID `0x01`:

1. **Start** (type `0x01`) — declares SSID length, password length, chunk counts, and checksum
2. **SSID chunks** (type `0x02`) — up to 6 bytes per message, indexed
3. **Password chunks** (type `0x03`) — up to 6 bytes per message, indexed
4. **End** (type `0x04`) — verifies XOR checksum and saves credentials to NVS

A 5-second timeout resets the state if the sequence is not completed.

### OTA Updates

The firmware supports over-the-air updates via WiFi. An OTA trigger message on CAN ID `0x00` with the device's MAC address (bytes 3-5) puts the device into OTA mode. The flash partition table supports dual-OTA (app0/app1) for safe rollback.

## Building

### Prerequisites

- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/) v5.x

### Build and Flash

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

## Project Structure

```
TrailCurrentSwitchback/
├── main/
│   ├── main.c              # Entry point — init and CAN task
│   ├── board.h             # Pin definitions for ESP32-S3-ETH-8DI-8RO-C
│   ├── relay.h / relay.c   # TCA9554 I2C relay driver (8 channels)
│   ├── can_handler.h / .c  # CAN protocol and message handling
│   └── wifi_config.h / .c  # WiFi credential storage and CAN provisioning
├── CMakeLists.txt          # Top-level ESP-IDF project file
├── partitions.csv          # Custom flash partition table (dual OTA)
└── sdkconfig.defaults      # Default SDK configuration
```

## License

See [LICENSE](LICENSE) for details.
