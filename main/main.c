/*
 * leon-1p-can: ΜΟΝΟ CAN sniffer.
 * - Καμία οθόνη, κανένα LVGL.
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
 * Αυτόματη αναζήτηση χρονισμού (sync).
 * Ρολόι CAN = 40 MHz. Για 500 kbit/s: 40 MHz / brp = quanta, bit = 1 + tseg1 + tseg2.
 * Δοκιμάζουμε κάθε ρύθμιση για SCAN_MS, μόνο ακρόαση, και κρατάμε την καλύτερη.
 */
#define SCAN_MS 3000

typedef struct {
    const char *name;
    twai_timing_advanced_config_t t;
} timing_t;

static const timing_t s_timings[] = {
    { "20tq SP70% SJW4",  { .brp = 4, .tseg_1 = 13, .tseg_2 = 6, .sjw = 4 } },
    { "20tq SP75% SJW4",  { .brp = 4, .tseg_1 = 14, .tseg_2 = 5, .sjw = 4 } },
    { "20tq SP80% SJW4",  { .brp = 4, .tseg_1 = 15, .tseg_2 = 4, .sjw = 4 } },
    { "20tq SP85% SJW3",  { .brp = 4, .tseg_1 = 16, .tseg_2 = 3, .sjw = 3 } },
    { "16tq SP75% SJW4",  { .brp = 5, .tseg_1 = 11, .tseg_2 = 4, .sjw = 4 } },
    { "16tq SP81% SJW3",  { .brp = 5, .tseg_1 = 12, .tseg_2 = 3, .sjw = 3 } },
    { "16tq SP87% SJW2",  { .brp = 5, .tseg_1 = 13, .tseg_2 = 2, .sjw = 2 } },
    { "10tq SP70% SJW3",  { .brp = 8, .tseg_1 = 6,  .tseg_2 = 3, .sjw = 3 } },
    { "10tq SP80% SJW2",  { .brp = 8, .tseg_1 = 7,  .tseg_2 = 2, .sjw = 2 } },
};
#define N_TIMINGS (sizeof(s_timings) / sizeof(s_timings[0]))

static void apply_timing(twai_node_handle_t node, const timing_t *t)
{
    twai_node_disable(node);
    ESP_ERROR_CHECK(twai_node_reconfig_timing(node, &t->t, NULL));
    ESP_ERROR_CHECK(twai_node_enable(node));
}

static void reset_counters(void)
{
    portENTER_CRITICAL(&s_lock);
    s_total = 0;
    s_err_stuff = s_err_form = s_err_bit = s_err_other = 0;
    portEXIT_CRITICAL(&s_lock);
}

static const timing_t *scan_timings(twai_node_handle_t node)
{
    printf("\n=== SYNC: testing %d timings, %d s each ===\n", (int)N_TIMINGS, SCAN_MS / 1000);
    int best = -1;
    uint32_t best_msgs = 0, best_errs = UINT32_MAX;
    for (int i = 0; i < (int)N_TIMINGS; i++) {
        apply_timing(node, &s_timings[i]);
        vTaskDelay(pdMS_TO_TICKS(200));
        reset_counters();
        vTaskDelay(pdMS_TO_TICKS(SCAN_MS));
        portENTER_CRITICAL(&s_lock);
        uint32_t msgs = s_total;
        uint32_t errs = s_err_stuff + s_err_form + s_err_bit + s_err_other;
        portEXIT_CRITICAL(&s_lock);
        uint32_t msgs_s = msgs * 1000 / SCAN_MS, errs_s = errs * 1000 / SCAN_MS;
        printf("  %d) %-18s -> %5" PRIu32 " msg/s, %6" PRIu32 " errors/s\n", i + 1, s_timings[i].name, msgs_s, errs_s);
        if (msgs > best_msgs || (msgs == best_msgs && errs < best_errs)) {
            best = i;
            best_msgs = msgs;
            best_errs = errs;
        }
    }
    if (best < 0) {
        best = 2;   /* τίποτα δεν ήρθε: μένουμε στο 80% */
    }
    printf("=== BEST: %d) %s ===\n", best + 1, s_timings[best].name);
    reset_counters();
    return &s_timings[best];
}

static int by_id(const void *a, const void *b)
{
    const slot_t *x = a, *y = b;
    return (x->id > y->id) - (x->id < y->id);
}

void app_main(void)
{
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

    const timing_t *best = scan_timings(node);
    apply_timing(node, best);

    printf("\n=== leon-1p-can sniffer: 500 kbit/s, %s, LISTEN-ONLY, TX=GPIO%d RX=GPIO%d ===\n",
           best->name, CAN_TX_GPIO, CAN_RX_GPIO);

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

        printf("\n   ID   /s  DLC  DATA\n");
        for (int i = 0; i < n; i++) {
            slot_t *s = &snap[i];
            printf(s->ext ? "%08" PRIX32 : "     %03" PRIX32, s->id);
            printf(" %4" PRIu32 "  [%u] ", s->count, s->len);
            for (int b = 0; b < s->len; b++) {
                printf(" %02X", s->data[b]);
            }
            printf("\n");
        }

        twai_node_status_t st = {0};
        twai_node_record_t rec = {0};
        twai_node_get_info(node, &st, &rec);
        uint32_t errs = e_stuff + e_form + e_bit + e_other;
        printf("--- %" PRIu32 " msg/s | IDs: %d | errors/s: %" PRIu32
               " (stuff %" PRIu32 ", form %" PRIu32 ", bit %" PRIu32 ", other %" PRIu32 ")"
               " | total errors: %" PRIu32 " | state: %s%s ---\n",
               total, n, errs, e_stuff, e_form, e_bit, e_other,
               rec.bus_err_num, state_name(st.state), full ? " | ID table full" : "");
        if (st.state == TWAI_ERROR_BUS_OFF) {
            twai_node_recover(node);
        }
    }
}
