idf_component_register(
    SRCS "main.c" "obd.c" "ui.c" "display.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer freertos
)
