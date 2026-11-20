/*
 * leon-1p-can: ΜΟΝΟ CAN sniffer.
 * - Καμία οθόνη, κανένα LVGL.
 * - CAN 500 kbit/s, μόνο ακρόαση (δεν στέλνει τίποτα στο αυτοκίνητο).
 * - Τυπώνει κάθε μήνυμα στο serial (921600 baud):  ID  [DLC]  data
 * - Κάθε 1 δευτερόλεπτο τυπώνει στατιστικά: μηνύματα/s, σφάλματα, κατάσταση.
 */
#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_timer.h"

#define CAN_TX_GPIO 2
#define CAN_RX_GPIO 3
#define CAN_BITRATE 500000

typedef struct {
    uint32_t id;
    uint8_t  ext;
    uint8_t  len;
    uint8_t  data[8];
} msg_t;

static QueueHandle_t s_q;
static volatile uint32_t s_rx, s_dropped, s_err_events;

static bool on_rx(twai_node_handle_t node, const twai_rx_done_event_data_t *e, void *ctx)
{
    msg_t m = {0};
    twai_frame_t f = { .buffer = m.data, .buffer_len = sizeof(m.data) };
    if (twai_node_receive_from_isr(node, &f) != ESP_OK) {
        return false;
    }
    m.id = f.header.id;
    m.ext = f.header.ide;
    m.len = f.header.dlc > 8 ? 8 : f.header.dlc;
    s_rx++;
    BaseType_t woken = pdFALSE;
    if (xQueueSendFromISR(s_q, &m, &woken) != pdTRUE) {
        s_dropped++;
    }
    return woken == pdTRUE;
}

static bool on_error(twai_node_handle_t node, const twai_error_event_data_t *e, void *ctx)
{
    s_err_events++;
    return false;
}

static const char *state_name(twai_error_state_t s)
{
    switch (s) {
    case TWAI_ERROR_ACTIVE:  return "OK (active)";
    case TWAI_ERROR_WARNING: return "WARNING";
    case TWAI_ERROR_PASSIVE: return "PASSIVE";
    case TWAI_ERROR_BUS_OFF: return "BUS OFF";
    default:                 return "?";
    }
}

void app_main(void)
{
    s_q = xQueueCreate(512, sizeof(msg_t));

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
    ESP_ERROR_CHECK(twai_node_enable(node));

    printf("\n=== leon-1p-can sniffer: %d kbit/s, LISTEN-ONLY, TX=GPIO%d RX=GPIO%d ===\n",
           CAN_BITRATE / 1000, CAN_TX_GPIO, CAN_RX_GPIO);

    int64_t next_stats = esp_timer_get_time() + 1000000;
    uint32_t last_rx = 0;
    msg_t m;
    for (;;) {
        if (xQueueReceive(s_q, &m, pdMS_TO_TICKS(100)) == pdTRUE) {
            printf(m.ext ? "%08" PRIX32 " [%u]" : "     %03" PRIX32 " [%u]", m.id, m.len);
            for (int i = 0; i < m.len; i++) {
                printf(" %02X", m.data[i]);
            }
            printf("\n");
        }

        int64_t now = esp_timer_get_time();
        if (now >= next_stats) {
            next_stats = now + 1000000;
            twai_node_status_t st = {0};
            twai_node_record_t rec = {0};
            twai_node_get_info(node, &st, &rec);
            uint32_t rx = s_rx;
            printf("--- %" PRIu32 " msg/s | bus errors: %" PRIu32 " | rx err count: %u | state: %s | dropped: %" PRIu32 " ---\n",
                   rx - last_rx, rec.bus_err_num, st.rx_error_count, state_name(st.state), s_dropped);
            last_rx = rx;
            if (st.state == TWAI_ERROR_BUS_OFF) {
                twai_node_recover(node);
            }
        }
    }
}
