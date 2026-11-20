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

    ESP_ERROR_CHECK(twai_node_reconfig_timing(node, &s_timing, NULL));
    ESP_ERROR_CHECK(twai_node_enable(node));

    printf("\n=== leon-1p-can sniffer: 500 kbit/s, SP70%% SJW4, LISTEN-ONLY, TX=GPIO%d RX=GPIO%d ===\n",
           CAN_TX_GPIO, CAN_RX_GPIO);

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
