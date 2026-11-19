#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "lvgl.h"
#include "display.h"
#include "obd.h"
#include "ui.h"

static uint32_t tick_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

void app_main(void)
{
    lv_init();
    lv_tick_set_cb(tick_ms);
    display_init();
    ui_create();

    obd_start();

    for (;;) {
        uint32_t wait = lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(wait < 5 ? 5 : (wait > 30 ? 30 : wait)));
    }
}
