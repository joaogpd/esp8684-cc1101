idf_component_register(
    SRCS "main.c" "bridge.c"
    INCLUDE_DIRS "."
    REQUIRES cc1101 ble_nus driver esp_timer nvs_flash vfs
)
