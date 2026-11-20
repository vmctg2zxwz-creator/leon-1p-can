/*
 * leon-1p-can: ΜΟΝΟ CAN sniffer.
 * - Η οθόνη δείχνει ακριβώς το ίδιο κείμενο με το serial (μόνο κείμενο).
 * - CAN 500 kbit/s, μόνο ακρόαση (δεν στέλνει τίποτα στο αυτοκίνητο).
 * - Χρονισμός σταθερός: SP65 (βρέθηκε με scan στο αυτοκίνητο, 0 err/s).
 * - Προστασία: αν έρθουν πάρα πολλά σφάλματα, το CAN σταματά για 2 s.
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
#include "esp_system.h"

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
static volatile uint32_t s_err_all;   /* σύνολο, για την προστασία */
static volatile uint32_t s_pauses;
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
    s_err_all++;
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
 * Χρονισμός 500 kbit/s, ρολόι CAN 40 MHz (XTAL): 40 MHz / 4 -> 20 quanta ανά bit = 1 + 12 + 7.
 * Sample point 65%, SJW 4. Scan στο αυτοκίνητο (20.11.2026):
 *   SP65 1270 msg/s 0 err/s | SP70 1266/20 | SP75 1189/385 | SP80 -> error flood
 */
typedef struct {
    const char *name;
    twai_timing_advanced_config_t t;
} timing_t;

static const timing_t s_timing = { "20tq SP65 SJW4", { .brp = 4, .tseg_1 = 12, .tseg_2 = 7, .sjw = 4 } };

static twai_node_handle_t s_node;

/* Φτιάχνει από την αρχή τον CAN node με τον δοσμένο χρονισμό (μόνο ακρόαση). */
static void can_open(const timing_t *t)
{
    if (s_node) {
        twai_node_disable(s_node);
        twai_node_delete(s_node);
        s_node = NULL;
    }
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
    ESP_ERROR_CHECK(twai_new_node_onchip(&cfg, &s_node));
    ESP_ERROR_CHECK(twai_node_reconfig_timing(s_node, &t->t, NULL));
    twai_event_callbacks_t cbs = { .on_rx_done = on_rx, .on_error = on_error };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(s_node, &cbs, NULL));
    ESP_ERROR_CHECK(twai_node_enable(s_node));
}

static void reset_counters(void)
{
    portENTER_CRITICAL(&s_lock);
    s_total = 0;
    s_err_stuff = s_err_form = s_err_bit = s_err_other = 0;
    for (int i = 0; i < s_used; i++) {
        s_tab[i].count = 0;
    }
    portEXIT_CRITICAL(&s_lock);
}

static int by_id(const void *a, const void *b)
{
    const slot_t *x = a, *y = b;
    return (x->id > y->id) - (x->id < y->id);
}

/* ---------------- Οθόνη: μόνο κείμενο ---------------- */

/* Η πλακέτα είναι τοποθετημένη οριζόντια: γυρνάμε το κείμενο 90°.
 * Αν βγει ανάποδα, άλλαξε σε 2700. */
#define TEXT_ROTATION 900

static lv_obj_t *s_label;
static char s_txt[4096];

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
    lv_obj_set_style_transform_scale(s_label, 384, 0);       /* 1.5x */
    lv_obj_set_style_transform_rotation(s_label, TEXT_ROTATION, 0);
    lv_obj_set_style_transform_pivot_x(s_label, 0, 0);
    lv_obj_set_style_transform_pivot_y(s_label, 0, 0);
#if TEXT_ROTATION == 900
    lv_obj_set_pos(s_label, 792, 8);          /* ξεκινά πάνω δεξιά */
#else
    lv_obj_set_pos(s_label, 8, 1272);         /* ξεκινά κάτω αριστερά */
#endif
    lv_label_set_text(s_label, "leon-1p-can");
    bsp_display_unlock();

    vTaskDelay(pdMS_TO_TICKS(100));
    bsp_display_backlight_on();
}

/* Στέλνει το s_txt στο serial και στην οθόνη. */
static void show(void)
{
    printf("\n%s", s_txt);
    if (bsp_display_lock(100) == ESP_OK) {
        lv_label_set_text(s_label, s_txt);
        bsp_display_unlock();
    }
}

#define ADD(len, ...) do { if ((len) < (int)sizeof(s_txt)) (len) += snprintf(s_txt + (len), sizeof(s_txt) - (len), __VA_ARGS__); } while (0)

/* ---------------- Προστασία από πλημμύρα σφαλμάτων ---------------- */

#define GUARD_MS        50
#define GUARD_MAX_ERRS  200     /* > 200 σφάλματα σε 50 ms (= 4000/s) -> παύση */
#define GUARD_PAUSE_MS  2000

static void guard_task(void *arg)
{
    uint32_t last = s_err_all;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(GUARD_MS));
        uint32_t now = s_err_all;
        if (now - last > GUARD_MAX_ERRS) {
            twai_node_disable(s_node);
            s_pauses++;
            printf("!!! CAN error flood (%" PRIu32 " in %d ms): paused %d ms\n",
                   now - last, GUARD_MS, GUARD_PAUSE_MS);
            vTaskDelay(pdMS_TO_TICKS(GUARD_PAUSE_MS));
            twai_node_enable(s_node);
            now = s_err_all;
        }
        last = now;
    }
}

static const char *reset_name(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON:  return "power on";
    case ESP_RST_SW:       return "software";
    case ESP_RST_PANIC:    return "PANIC (crash)";
    case ESP_RST_INT_WDT:  return "INTERRUPT WATCHDOG";
    case ESP_RST_TASK_WDT: return "TASK WATCHDOG";
    case ESP_RST_WDT:      return "WATCHDOG";
    case ESP_RST_BROWNOUT: return "BROWNOUT (low voltage)";
    case ESP_RST_EXT:      return "reset button";
    default:               return "other";
    }
}

void app_main(void)
{
    const char *boot = reset_name(esp_reset_reason());
    screen_init();

    const timing_t *best = &s_timing;
    can_open(best);
    reset_counters();
    xTaskCreate(guard_task, "can_guard", 3072, NULL, 10, NULL);

    static slot_t snap[MAX_IDS];
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
        twai_node_get_info(s_node, &st, &rec);
        uint32_t errs = e_stuff + e_form + e_bit + e_other;

        int len = 0;
        ADD(len, "timing: %s   last reset: %s   pauses: %" PRIu32 "\n", best->name, boot, s_pauses);
        ADD(len, "   ID   /s DLC DATA\n");
        for (int i = 0; i < n; i++) {
            slot_t *s = &snap[i];
            if (s->ext) {
                ADD(len, "%08" PRIX32, s->id);
            } else {
                ADD(len, "     %03" PRIX32, s->id);
            }
            ADD(len, " %4" PRIu32 " [%u]", s->count, s->len);
            for (int b = 0; b < s->len; b++) {
                ADD(len, " %02X", s->data[b]);
            }
            ADD(len, "\n");
        }
        ADD(len, "msg/s %" PRIu32 "   IDs %d\n", total, n);
        ADD(len, "err/s %" PRIu32 " (stuff %" PRIu32 " form %" PRIu32 " bit %" PRIu32 " other %" PRIu32 ")\n",
            errs, e_stuff, e_form, e_bit, e_other);
        ADD(len, "total err %" PRIu32 "   state %s%s\n", rec.bus_err_num, state_name(st.state),
            full ? "   ID TABLE FULL" : "");
        show();

        if (st.state == TWAI_ERROR_BUS_OFF) {
            twai_node_recover(s_node);
        }
    }
}
