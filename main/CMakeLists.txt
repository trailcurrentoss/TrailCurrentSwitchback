idf_component_register(
    SRCS "ota.c" "discovery.c" "main.c" "relay.c" "can_handler.c" "wifi_config.c" "gnss.c" "sensors.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer nvs_flash esp_wifi esp_netif esp_eth esp_http_server esp_app_format app_update esp_adc can_common
)

# Module address (0-2): idf.py build -DSWITCHBACK_ADDRESS=1
if(DEFINED SWITCHBACK_ADDRESS)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SWITCHBACK_ADDRESS=${SWITCHBACK_ADDRESS})
endif()

# Variant selector: base | picket | aftline | picket_gnss
#   base        — relay-only, no DI CAN broadcast
#   picket      — 5 Hz PicketStatus8-10 (0x12-0x14), 2-byte DoorStatus format
#   aftline     — 30 Hz TrailerStatus (0x3A), 3-byte flags + trailer voltage
#   picket_gnss — picket DI broadcast + Bearing-format GNSS frames (0x06-0x09)
#                 + Borealis-format EnvironmentalStatus (0x1F) driven by an
#                 SCD41 on the shared I2C bus. Wires the DFRobot Gravity GNSS
#                 over UART on GPIO 1/2 and the SCD41 (fixed 0x62) alongside
#                 the TCA9554 relay expander.
# Default is picket, preserving existing behavior when the flag is omitted.
if(NOT DEFINED SWITCHBACK_VARIANT)
    set(SWITCHBACK_VARIANT "picket")
endif()
if(SWITCHBACK_VARIANT STREQUAL "base")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SWITCHBACK_VARIANT_BASE=1)
elseif(SWITCHBACK_VARIANT STREQUAL "picket")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SWITCHBACK_VARIANT_PICKET=1)
elseif(SWITCHBACK_VARIANT STREQUAL "aftline")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SWITCHBACK_VARIANT_AFTLINE=1)
elseif(SWITCHBACK_VARIANT STREQUAL "picket_gnss")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SWITCHBACK_VARIANT_PICKET=1 SWITCHBACK_VARIANT_GNSS=1 SWITCHBACK_VARIANT_SCD41=1)
else()
    message(FATAL_ERROR "SWITCHBACK_VARIANT must be base, picket, aftline, or picket_gnss (got: ${SWITCHBACK_VARIANT})")
endif()
