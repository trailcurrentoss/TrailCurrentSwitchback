#!/bin/bash
# Build all Switchback address × variant combinations.
# Variants:
#   base    — relay-only, no DI CAN broadcast
#   picket  — 5 Hz PicketStatus8-10 (0x12-0x14), 2-byte DoorStatus format
#   aftline — 30 Hz TrailerStatus (0x3A), 3-byte flags + trailer voltage
#
# Produces two binaries per (address, variant):
#   build/switchback_addr{N}_{variant}.bin         — app-only (for OTA via Headwaters)
#   build/switchback_addr{N}_{variant}_merged.bin  — merged  (for web flasher, full flash at 0x0)
#
# The app-only binary contains just the application image. Headwaters OTA
# writes it to a single app partition via esp_ota_write, which validates
# the image as an app. A merged binary would fail that validation because
# it starts with the bootloader, not an app header.
#
# The merged binary combines bootloader + partition table + OTA data + app
# into one file flashable at offset 0x0. The web flasher requires this
# because it writes the entire flash from a single binary.
set -e

MAX_ADDR=2
VARIANTS=(base picket aftline)
OUTPUT_DIR="build"

for variant in "${VARIANTS[@]}"; do
    for addr in $(seq 0 $MAX_ADDR); do
        echo "========================================"
        echo "Building Switchback addr=$addr variant=$variant ..."
        echo "========================================"
        idf.py build -DSWITCHBACK_ADDRESS=$addr -DSWITCHBACK_VARIANT=$variant

        # Copy app-only binary with address+variant name (for OTA)
        cp "$OUTPUT_DIR/trailcurrent_switchback.bin" "$OUTPUT_DIR/switchback_addr${addr}_${variant}.bin"

        # Create merged binary (for web flasher — flashable at 0x0)
        esptool.py --chip esp32s3 merge_bin -o "$OUTPUT_DIR/switchback_addr${addr}_${variant}_merged.bin" \
            --flash_mode dio --flash_size 16MB \
            0x0 "$OUTPUT_DIR/bootloader/bootloader.bin" \
            0x8000 "$OUTPUT_DIR/partition_table/partition-table.bin" \
            0xe000 "$OUTPUT_DIR/ota_data_initial.bin" \
            0x10000 "$OUTPUT_DIR/trailcurrent_switchback.bin"
        echo ""
    done
done

echo "========================================"
echo "Build complete"
echo "========================================"
echo ""
echo "App-only binaries (for OTA):"
ls -lh "$OUTPUT_DIR"/switchback_addr[0-9]_*.bin 2>/dev/null | grep -v _merged
echo ""
echo "Merged binaries (for web flasher):"
ls -lh "$OUTPUT_DIR"/switchback_addr*_*_merged.bin
echo ""
echo "Attach ALL of the above to the GitHub release."
