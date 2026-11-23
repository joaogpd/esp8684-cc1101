idf_component_register(
    SRCS "cc1101.c" "cc1101_settings.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer
)
