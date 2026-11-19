#include "display.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

/*
 * ΠΡΟΣΩΡΙΝΟ: εδώ μπαίνει ο driver της συγκεκριμένης πλακέτας
 * (MIPI-DSI panel + backlight + touch). Μέχρι να ξέρουμε το μοντέλο,
 * το LVGL τρέχει κανονικά αλλά η εικόνα δεν στέλνεται πουθενά.
 */

static const char *TAG = "display";

static void flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px)
{
    /* TODO: esp_lcd_panel_draw_bitmap(panel, area->x1, area->y1, area->x2 + 1, area->y2 + 1, px); */
    lv_display_flush_ready(disp);
}

lv_display_t *display_init(void)
{
    const size_t buf_px = DISP_HOR_RES * 60;
    const size_t buf_bytes = buf_px * sizeof(uint16_t);
    void *buf1 = heap_caps_malloc(buf_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    void *buf2 = heap_caps_malloc(buf_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    assert(buf1 && buf2);

    lv_display_t *disp = lv_display_create(DISP_HOR_RES, DISP_VER_RES);
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(disp, buf1, buf2, buf_bytes, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(disp, flush_cb);

    ESP_LOGW(TAG, "Placeholder display driver: δεν έχει οριστεί πλακέτα ακόμα");
    return disp;
}
