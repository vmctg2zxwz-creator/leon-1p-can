/*
 * leon-1p-can: ΜΟΝΟ CAN sniffer.
 * - Η οθόνη δείχνει ακριβώς το ίδιο κείμενο με το serial (μόνο κείμενο).
 * - CAN 500 kbit/s, μόνο ακρόαση (δεν στέλνει τίποτα στο αυτοκίνητο).
 * - Serial 115200 baud. Κάθε 1 δευτερόλεπτο τυπώνει έναν πίνακα:
 *   μία γραμμή για κάθε ID με πόσα μηνύματα ήρθαν και τα τελευταία δεδομένα,
 *   και στο τέλος μία γραμμή με στατιστικά (σφάλματα, κατάσταση).
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "lvgl.h"

#define CAN_TX_GPIO 2
#define CAN_RX_GPIO 3
#define CAN_BITRATE 500000
#define MAX_IDS     128

typedef struct {
    uint32_t id;
    uint8_t  ext;
    uint8_t  len;
    uint8_t  data[8];
    uint32_t count;     /* μηνύματα από την τελευταία εκτύπωση */
} slot_t;

static slot_t s_tab[MAX_IDS];
static int s_used;
static uint32_t s_total, s_full;
static uint32_t s_err_stuff, s_err_form, s_err_bit, s_err_other;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static bool on_rx(twai_node_handle_t node, const twai_rx_done_event_data_t *e, void *ctx)
{
    uint8_t d[8] = {0};
    twai_frame_t f = { .buffer = d, .buffer_len = sizeof(d) };
    if (twai_node_receive_from_isr(node, &f) != ESP_OK) {
        return false;
    }
    uint8_t len = f.header.dlc > 8 ? 8 : f.header.dlc;

    portENTER_CRITICAL_ISR(&s_lock);
    s_total++;
    slot_t *s = NULL;
    for (int i = 0; i < s_used; i++) {
        if (s_tab[i].id == f.header.id && s_tab[i].ext == f.header.ide) {
            s = &s_tab[i];
            break;
        }
    }
    if (!s && s_used < MAX_IDS) {
        s = &s_tab[s_used++];
        s->id = f.header.id;
        s->ext = f.header.ide;
    }
    if (s) {
        s->len = len;
        memcpy(s->data, d, sizeof(d));
        s->count++;
    } else {
        s_full++;
    }
    portEXIT_CRITICAL_ISR(&s_lock);
    return false;
}

static bool on_error(twai_node_handle_t node, const twai_error_event_data_t *e, void *ctx)
{
    portENTER_CRITICAL_ISR(&s_lock);
    if (e->err_flags.stuff_err) s_err_stuff++;
    else if (e->err_flags.form_err) s_err_form++;
    else if (e->err_flags.bit_err) s_err_bit++;
    else s_err_other++;
    portEXIT_CRITICAL_ISR(&s_lock);
    return false;
}

static const char *state_name(twai_error_state_t s)
{
    switch (s) {
    case TWAI_ERROR_ACTIVE:  return "OK";
    case TWAI_ERROR_WARNING: return "WARNING";
    case TWAI_ERROR_PASSIVE: return "PASSIVE";
    case TWAI_ERROR_BUS_OFF: return "BUS OFF";
    default:                 return "?";
    }
}

/*
 * Χρονισμός 500 kbit/s, βρέθηκε με scan στο αυτοκίνητο (20.11.2026):
 * 40 MHz / 4 = 10 MHz -> 20 quanta ανά bit = 1 + 13 + 6.
 * Sample point 70%, SJW 4 -> ~1260 msg/s, ~60 errors/s.
 */
static const twai_timing_advanced_config_t s_timing = {
    .brp = 4,
    .prop_seg = 0,
    .tseg_1 = 13,
    .tseg_2 = 6,
    .sjw = 4,
};

static int by_id(const void *a, const void *b)
{
    const slot_t *x = a, *y = b;
    return (x->id > y->id) - (x->id < y->id);
}

static lv_obj_t *s_label;

/* Οθόνη: μαύρο φόντο, ένα label με σταθερού πλάτους γράμματα, 2x μέγεθος. */
static void screen_init(void)
{
    bsp_display_cfg_t cfg = {
        .lv_adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG(),
        .rotation = ESP_LV_ADAPTER_ROTATE_0,
        .tear_avoid_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_TRIPLE_PARTIAL,
    };
    bsp_display_start_with_config(&cfg);

    bsp_display_lock(-1);
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    s_label = lv_label_create(scr);
    lv_obj_set_style_text_font(s_label, &lv_font_unscii_16, 0);
    lv_obj_set_style_text_color(s_label, lv_color_hex(0x00ff66), 0);
    lv_obj_set_style_transform_scale(s_label, 512, 0);
    lv_obj_set_style_transform_pivot_x(s_label, 0, 0);
    lv_obj_set_style_transform_pivot_y(s_label, 0, 0);
    lv_obj_set_pos(s_label, 8, 8);
    lv_label_set_text(s_label, "leon-1p-can\nwaiting for CAN...");
    bsp_display_unlock();

    /* Backlight μία φορά, αφού έχει ζωγραφιστεί η πρώτη εικόνα. */
    vTaskDelay(pdMS_TO_TICKS(100));
    bsp_display_backlight_on();
}

void app_main(void)
{
    screen_init();

    twai_onchip_node_config_t cfg = {
        .io_cfg = {
            .tx = CAN_TX_GPIO,
            .rx = CAN_RX_GPIO,
            .quanta_clk_out = -1,
            .bus_off_indicator = -1,
        },
        .bit_timing = { .bitrate = CAN_BITRATE },
        .tx_queue_depth = 1,
        .flags = { .enable_listen_only = true },
    };
    twai_node_handle_t node;
    ESP_ERROR_CHECK(twai_new_node_onchip(&cfg, &node));
    twai_event_callbacks_t cbs = { .on_rx_done = on_rx, .on_error = on_error };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(node, &cbs, NULL));

    ESP_ERROR_CHECK(twai_node_reconfig_timing(node, &s_timing, NULL));
    ESP_ERROR_CHECK(twai_node_enable(node));

    printf("\n=== leon-1p-can sniffer: 500 kbit/s, SP70%% SJW4, LISTEN-ONLY, TX=GPIO%d RX=GPIO%d ===\n",
           CAN_TX_GPIO, CAN_RX_GPIO);

    static slot_t snap[MAX_IDS];
    static char txt[4096];
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        portENTER_CRITICAL(&s_lock);
        int n = s_used;
        memcpy(snap, s_tab, n * sizeof(slot_t));
        for (int i = 0; i < n; i++) {
            s_tab[i].count = 0;
        }
        uint32_t total = s_total, full = s_full;
        uint32_t e_stuff = s_err_stuff, e_form = s_err_form, e_bit = s_err_bit, e_other = s_err_other;
        s_total = 0;
        s_err_stuff = s_err_form = s_err_bit = s_err_other = 0;
        portEXIT_CRITICAL(&s_lock);

        qsort(snap, n, sizeof(slot_t), by_id);

        twai_node_status_t st = {0};
        twai_node_record_t rec = {0};
        twai_node_get_info(node, &st, &rec);
        uint32_t errs = e_stuff + e_form + e_bit + e_other;

        /* Το ίδιο κείμενο πάει στο serial και στην οθόνη. */
        int len = 0;
#define ADD(...) do { if (len < (int)sizeof(txt)) len += snprintf(txt + len, sizeof(txt) - len, __VA_ARGS__); } while (0)
        ADD("   ID   /s DLC DATA\n");
        for (int i = 0; i < n; i++) {
            slot_t *s = &snap[i];
            if (s->ext) {
                ADD("%08" PRIX32, s->id);
            } else {
                ADD("     %03" PRIX32, s->id);
            }
            ADD(" %4" PRIu32 " [%u]", s->count, s->len);
            for (int b = 0; b < s->len; b++) {
                ADD(" %02X", s->data[b]);
            }
            ADD("\n");
        }
        ADD("\nmsg/s %" PRIu32 "   IDs %d\n", total, n);
        ADD("err/s %" PRIu32 " (stuff %" PRIu32 " form %" PRIu32 " bit %" PRIu32 " other %" PRIu32 ")\n",
            errs, e_stuff, e_form, e_bit, e_other);
        ADD("total err %" PRIu32 "   state %s%s\n", rec.bus_err_num, state_name(st.state),
            full ? "   ID TABLE FULL" : "");
#undef ADD

        printf("\n%s", txt);

        if (bsp_display_lock(100) == ESP_OK) {
            lv_label_set_text(s_label, txt);
            bsp_display_unlock();
        }

        if (st.state == TWAI_ERROR_BUS_OFF) {
            twai_node_recover(node);
        }
    }
}
