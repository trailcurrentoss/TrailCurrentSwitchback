#!/bin/bash
# Build all Switchback address variants (0-2).
# Produces two binaries per address:
#   build/switchback_addr{N}.bin         — app-only (for OTA via Headwaters)
#   build/switchback_addr{N}_merged.bin  — merged  (for web flasher, full flash at 0x0)
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
OUTPUT_DIR="build"

for addr in $(seq 0 $MAX_ADDR); do
    echo "========================================"
    echo "Building Switchback address $addr ..."
    echo "========================================"
    idf.py build -DSWITCHBACK_ADDRESS=$addr

    # Copy app-only binary with address-specific name (for OTA)
    cp "$OUTPUT_DIR/trailcurrent_switchback.bin" "$OUTPUT_DIR/switchback_addr${addr}.bin"

    # Create merged binary (for web flasher — flashable at 0x0)
    esptool.py --chip esp32s3 merge_bin -o "$OUTPUT_DIR/switchback_addr${addr}_merged.bin" \
        --flash_mode dio --flash_size 8MB \
        0x0 "$OUTPUT_DIR/bootloader/bootloader.bin" \
        0x8000 "$OUTPUT_DIR/partition_table/partition-table.bin" \
        0xe000 "$OUTPUT_DIR/ota_data_initial.bin" \
        0x10000 "$OUTPUT_DIR/trailcurrent_switchback.bin"
    echo ""
done

echo "========================================"
echo "Build complete"
echo "========================================"
echo ""
echo "App-only binaries (for OTA):"
ls -lh "$OUTPUT_DIR"/switchback_addr[0-9].bin
echo ""
echo "Merged binaries (for web flasher):"
ls -lh "$OUTPUT_DIR"/switchback_addr*_merged.bin
echo ""
echo "Attach ALL of the above to the GitHub release."
