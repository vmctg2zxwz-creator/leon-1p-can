idf_component_register(
    SRCS "main.c" "can.c" "ui.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_twai esp32_p4_platform
)
