#!/bin/bash
# Build all Switchback address variants (0-2).
# Produces: build/switchback_addr0.bin, switchback_addr1.bin, switchback_addr2.bin
set -e

MAX_ADDR=2
OUTPUT_DIR="build"

for addr in $(seq 0 $MAX_ADDR); do
    echo "========================================"
    echo "Building Switchback address $addr ..."
    echo "========================================"
    idf.py build -DSWITCHBACK_ADDRESS=$addr
    cp "$OUTPUT_DIR/trailcurrent_switchback.bin" "$OUTPUT_DIR/switchback_addr${addr}.bin"
    echo ""
done

echo "========================================"
echo "Build complete"
echo "========================================"
ls -lh "$OUTPUT_DIR"/switchback_addr*.bin
