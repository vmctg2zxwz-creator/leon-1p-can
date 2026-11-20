/*
 * leon-1p-can: ΜΟΝΟ CAN sniffer.
 * - Η οθόνη δείχνει ακριβώς το ίδιο κείμενο με το serial (μόνο κείμενο).
 * - CAN 500 kbit/s, μόνο ακρόαση (δεν στέλνει τίποτα στο αυτοκίνητο).
 * - Στην εκκίνηση δοκιμάζει 9 χρονισμούς (SCAN_MS ο καθένας) και κρατάει
 *   αυτόν με τα λιγότερα σφάλματα.
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
 * Χρονισμοί για 500 kbit/s. Ρολόι CAN = 40 MHz (XTAL).
 * bit = 1 + tseg1 + tseg2 quanta, quanta = brp / 40 MHz.
 */
typedef struct {
    const char *name;
    twai_timing_advanced_config_t t;
} timing_t;

static const timing_t s_timings[] = {
    { "20tq SP65 SJW4", { .brp = 4, .tseg_1 = 12, .tseg_2 = 7, .sjw = 4 } },
    { "20tq SP70 SJW4", { .brp = 4, .tseg_1 = 13, .tseg_2 = 6, .sjw = 4 } },
    { "20tq SP75 SJW4", { .brp = 4, .tseg_1 = 14, .tseg_2 = 5, .sjw = 4 } },
    { "20tq SP80 SJW4", { .brp = 4, .tseg_1 = 15, .tseg_2 = 4, .sjw = 4 } },
    { "16tq SP69 SJW4", { .brp = 5, .tseg_1 = 10, .tseg_2 = 5, .sjw = 4 } },
    { "16tq SP75 SJW4", { .brp = 5, .tseg_1 = 11, .tseg_2 = 4, .sjw = 4 } },
    { "10tq SP60 SJW4", { .brp = 8, .tseg_1 = 5,  .tseg_2 = 4, .sjw = 4 } },
    { "10tq SP70 SJW3", { .brp = 8, .tseg_1 = 6,  .tseg_2 = 3, .sjw = 3 } },
    { "10tq SP80 SJW2", { .brp = 8, .tseg_1 = 7,  .tseg_2 = 2, .sjw = 2 } },
};
#define N_TIMINGS ((int)(sizeof(s_timings) / sizeof(s_timings[0])))
#define SCAN_MS   5000

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

/* ---------------- Scan χρονισμών ---------------- */

static const timing_t *scan(void)
{
    static uint32_t msgs[N_TIMINGS], errs[N_TIMINGS], form[N_TIMINGS];
    int len = 0;
    s_txt[0] = 0;
    ADD(len, "SYNC: %d timings x %d s, LISTEN-ONLY\n", N_TIMINGS, SCAN_MS / 1000);
    show();

    for (int i = 0; i < N_TIMINGS; i++) {
        int mark = len;
        ADD(len, "%d) %-15s testing...\n", i + 1, s_timings[i].name);
        show();

        can_open(&s_timings[i]);
        vTaskDelay(pdMS_TO_TICKS(300));
        reset_counters();
        vTaskDelay(pdMS_TO_TICKS(SCAN_MS));

        portENTER_CRITICAL(&s_lock);
        msgs[i] = s_total;
        errs[i] = s_err_stuff + s_err_form + s_err_bit + s_err_other;
        form[i] = s_err_form;
        portEXIT_CRITICAL(&s_lock);

        len = mark;
        ADD(len, "%d) %-15s %5" PRIu32 " msg/s %5" PRIu32 " err/s\n", i + 1, s_timings[i].name,
            msgs[i] * 1000 / SCAN_MS, errs[i] * 1000 / SCAN_MS);
        show();
    }

    /* Καλύτερο: τα λιγότερα σφάλματα, ανάμεσα σε όσα πιάνουν >= 90% των μηνυμάτων. */
    uint32_t max_msgs = 0;
    for (int i = 0; i < N_TIMINGS; i++) {
        if (msgs[i] > max_msgs) max_msgs = msgs[i];
    }
    int best = 1;   /* αν δεν έρθει τίποτα: SP70 */
    if (max_msgs > 0) {
        uint32_t best_errs = UINT32_MAX;
        for (int i = 0; i < N_TIMINGS; i++) {
            if (msgs[i] * 10 >= max_msgs * 9 && errs[i] < best_errs) {
                best = i;
                best_errs = errs[i];
            }
        }
    }
    ADD(len, "BEST: %d) %s\n", best + 1, s_timings[best].name);
    show();
    vTaskDelay(pdMS_TO_TICKS(3000));
    return &s_timings[best];
}

void app_main(void)
{
    screen_init();

    const timing_t *best = scan();
    can_open(best);
    reset_counters();

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
        ADD(len, "timing: %s\n", best->name);
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
