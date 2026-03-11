# TrailCurrent Switchback

CAN-controlled 6-channel relay module for the TrailCurrent ecosystem, built on the [Waveshare ESP32-S3-Relay-6CH](https://www.waveshare.com/wiki/ESP32-S3-Relay-6CH).

## Hardware

### Base Module

The Waveshare ESP32-S3-Relay-6CH provides:

- **ESP32-S3** dual-core LX7 @ 240 MHz with WiFi and BLE
- **6 relay channels** (1NO/1NC each), rated 10A @ 250V AC / 10A @ 30V DC
- **7–36V DC** input (or 5V via USB-C)
- **RS485** isolated interface
- **DIN-rail mount** ABS enclosure (88 x 122 mm)
- Optocoupler isolation, TVS surge suppression, ceramic gas discharge tube protection

### CAN Hat (Coming Soon)

A simple PCB hat that sits on top of the Waveshare module's pin header and connects a CAN transceiver to the ESP32-S3 GPIO. The KiCad project files are in [EDA/TrailCurrentSwitchback/](EDA/TrailCurrentSwitchback/) and are currently scaffolding — schematic and layout are in progress.

**CAN bus pins:**

| Function | GPIO |
|----------|------|
| CAN TX   | 5    |
| CAN RX   | 4    |

## Firmware

### Pin Assignments

| Function   | GPIO |
|------------|------|
| Relay CH1  | 1    |
| Relay CH2  | 2    |
| Relay CH3  | 41   |
| Relay CH4  | 42   |
| Relay CH5  | 45   |
| Relay CH6  | 46   |
| Buzzer     | 21   |
| RGB LED    | 38   |
| CAN TX     | 4    |
| CAN RX     | 5    |

### CAN Protocol

CAN bus runs at **500 kbps** in no-ACK mode.

| CAN ID | Direction | Description |
|--------|-----------|-------------|
| `0x00` | RX | **OTA trigger** — 3-byte payload (MAC bytes 3–5) targets a specific device for WiFi OTA update |
| `0x01` | RX | **WiFi config** — multi-message chunked protocol to provision SSID and password over CAN |
| `0x25` | RX | **Relay toggle** — toggle a single relay (channel 0–5) or all relays at once |
| `0x28` | TX | **Status broadcast** — 1-byte bitmask (bits 0–5 = relay states), sent at ~30 Hz |

### WiFi Provisioning over CAN

WiFi credentials can be sent to the device over the CAN bus using a multi-message protocol on CAN ID `0x01`:

1. **Start** (type `0x01`) — declares SSID length, password length, chunk counts, and checksum
2. **SSID chunks** (type `0x02`) — up to 6 bytes per message, indexed
3. **Password chunks** (type `0x03`) — up to 6 bytes per message, indexed
4. **End** (type `0x04`) — verifies XOR checksum and saves credentials to NVS

A 5-second timeout resets the state if the sequence is not completed.

### OTA Updates

The firmware supports over-the-air updates via WiFi. An OTA trigger message on CAN ID `0x00` with the device's MAC address (bytes 3–5) puts the device into OTA mode with a 3-minute timeout. The flash partition table supports dual-OTA (app0/app1) for safe rollback.

## Building

### Prerequisites

- [PlatformIO](https://platformio.org/)

### Build and Flash

```bash
pio run                    # build
pio run -t upload          # flash via USB
pio run -t monitor         # serial monitor (115200 baud)
```

### Dependencies

Pulled automatically by PlatformIO from GitHub:

- **ESP32ArduinoDebugLibrary** — debug output macros
- **TwaiTaskBasedLibraryWROOM32** — CAN (TWAI) bus driver
- **OtaUpdateLibraryWROOM32** — WiFi OTA update mechanism

## Project Structure

```
TrailCurrentSwitchback/
├── src/
│   ├── main.cpp          # Entry point — init and loop
│   ├── globals.h         # Pin definitions and relay state
│   ├── canHelper.h       # CAN protocol and relay control
│   └── wifiConfig.h      # WiFi credential storage and CAN provisioning
├── EDA/
│   └── TrailCurrentSwitchback/   # KiCad project for CAN hat (WIP)
├── platformio.ini        # Build configuration
└── partitions.csv        # Custom flash partition table (dual OTA)
```

## License

See [LICENSE](LICENSE) for details.
