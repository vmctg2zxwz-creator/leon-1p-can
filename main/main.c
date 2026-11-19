#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "can.h"
#include "ui.h"

void app_main(void)
{
    /* Οθόνη 10.1" + LVGL από το BSP της Waveshare. */
    bsp_display_cfg_t cfg = {
        .lv_adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG(),
        .rotation = ESP_LV_ADAPTER_ROTATE_0,
        .tear_avoid_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_TRIPLE_PARTIAL,
    };
    bsp_display_start_with_config(&cfg);
    bsp_display_backlight_on();

    bsp_display_lock(-1);
    ui_create();
    bsp_display_unlock();

    can_start();
}
