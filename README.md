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
| `0x25 + addr` | RX | **Relay toggle** — toggle a single relay (channel 0-7) or set all relays (channel >= 8, byte 2 = on/off) |
| `0x28 + addr` | TX | **Status broadcast** — 1-byte bitmask (bits 0-7 = relay states), sent at ~30 Hz |

Up to 3 Switchback modules can share the same CAN bus. CAN IDs are offset by the `SWITCHBACK_ADDRESS` build flag:

| Base ID | Addr 0 | Addr 1 | Addr 2 | Description |
|---------|--------|--------|--------|-------------|
| 0x25 | 0x25 | 0x26 | 0x27 | Relay toggle (RX) |
| 0x28 | 0x28 | 0x29 | 0x2A | Status broadcast (TX) |

### WiFi Provisioning over CAN

WiFi credentials can be sent to the device over the CAN bus using a multi-message protocol on CAN ID `0x01`:

1. **Start** (type `0x01`) — declares SSID length, password length, chunk counts, and checksum
2. **SSID chunks** (type `0x02`) — up to 6 bytes per message, indexed
3. **Password chunks** (type `0x03`) — up to 6 bytes per message, indexed
4. **End** (type `0x04`) — verifies XOR checksum and saves credentials to NVS

A 5-second timeout resets the state if the sequence is not completed.

### OTA Updates

The firmware supports over-the-air updates via WiFi. An OTA trigger message on CAN ID `0x00` with the device's MAC address (bytes 3-5) puts the device into OTA mode. The flash partition table supports dual-OTA (app0/app1) for safe rollback.

Always use the app-only binary (`switchback_addr{N}.bin`) for OTA, never the merged binary. The merged binary starts with the bootloader, not an app header, and would fail the `esp_ota_end` validation.

```bash
curl -X POST http://esp32-XXYYZZ.local/ota --data-binary @build/switchback_addr0.bin
```

## Building

### Prerequisites

- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/) v5.x

### Build and Flash

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

### Multi-Instance Addressing

Up to 3 Switchback modules can share the same CAN bus. Each module is built with a unique address:

```bash
idf.py build                        # Address 0 (default)
idf.py build -DSWITCHBACK_ADDRESS=1  # Address 1
idf.py build -DSWITCHBACK_ADDRESS=2  # Address 2
```

#### Building All Variants

Use `build-all.sh` to build all 3 address variants in a single run:

```bash
./build-all.sh
```

This produces two binaries per address — one for OTA updates, one for the web flasher:

| File | Contents | Used By |
|------|----------|---------|
| `build/switchback_addr{N}.bin` | Application image only | Headwaters OTA (`deploy.sh`, `ota.js`), direct `curl` uploads |
| `build/switchback_addr{N}_merged.bin` | Bootloader + partition table + OTA data + application | Web flasher (full flash at 0x0) |

The two binary types exist because OTA and the web flasher write to different targets. Headwaters OTA sends the binary to the device's `/ota` HTTP endpoint, which calls `esp_ota_write` to write it to a single app partition. That function validates the image as an application — a merged binary starts with the bootloader instead of an app header, so it would fail validation. The web flasher writes the entire flash contents starting at offset 0x0, so it needs all partitions combined into one file.

#### Creating a GitHub Release

After building all variants, attach all 6 binaries (3 app-only + 3 merged) as release assets:

```bash
git tag -a v1.0.0 -m "Firmware release v1.0.0"
git push origin v1.0.0

gh release create v1.0.0 \
  build/switchback_addr0.bin \
  build/switchback_addr1.bin \
  build/switchback_addr2.bin \
  build/switchback_addr0_merged.bin \
  build/switchback_addr1_merged.bin \
  build/switchback_addr2_merged.bin \
  --repo trailcurrentoss/TrailCurrentSwitchback \
  --title "v1.0.0" \
  --notes "Firmware release v1.0.0"
```

Both the Headwaters deployment system and the web flasher pull from GitHub releases. The web flasher matches `_merged.bin` files by name for full-flash use. The Headwaters deployment system (`fetch-firmware.sh`) downloads the app-only `switchback_addr{N}.bin` files for OTA delivery.

## Project Structure

```
TrailCurrentSwitchback/
├── main/
│   ├── main.c              # Entry point — init and CAN task
│   ├── board.h             # Pin definitions for ESP32-S3-ETH-8DI-8RO-C
│   ├── relay.h / relay.c   # TCA9554 I2C relay driver (8 channels)
│   ├── can_handler.h / .c  # CAN protocol and message handling
│   ├── wifi_config.h / .c  # WiFi credential storage and CAN provisioning
│   ├── discovery.h / .c    # mDNS self-discovery and Headwaters registration
│   └── ota.h / ota.c       # OTA firmware updates via WiFi
├── CMakeLists.txt          # Top-level ESP-IDF project file
├── partitions.csv          # Custom flash partition table (dual OTA)
├── build-all.sh            # Build all 3 address variants (addr 0-2)
└── sdkconfig.defaults      # Default SDK configuration
```

## License

See [LICENSE](LICENSE) for details.
