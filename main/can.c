#include "can.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

/*
 * Leon 1P (PQ35) powertrain CAN, 500 kbit/s.
 * ΜΟΝΟ ακρόαση: η πλακέτα δεν στέλνει τίποτα στο αυτοκίνητο.
 */
#define CAN_TX_GPIO 2
#define CAN_RX_GPIO 3
#define STALE_MS    1000   /* μετά από 1s χωρίς μήνυμα δείχνουμε "--" */

#define ID_RPM      0x280  /* Motor_1 */
#define ID_COOLANT  0x288  /* Motor_2 */
#define ID_INTAKE   0x380  /* Motor_3 */

static const char *TAG = "can";
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static float s_rpm, s_coolant, s_intake;
static TickType_t s_t_rpm, s_t_coolant, s_t_intake;
static bool s_seen_rpm, s_seen_coolant, s_seen_intake;

/* Τρέχει μέσα στο interrupt για κάθε μήνυμα που φτάνει. */
static bool on_rx(twai_node_handle_t node, const twai_rx_done_event_data_t *e, void *ctx)
{
    uint8_t d[8] = {0};
    twai_frame_t f = { .buffer = d, .buffer_len = sizeof(d) };
    if (twai_node_receive_from_isr(node, &f) != ESP_OK || f.header.ide) {
        return false;
    }
    TickType_t now = xTaskGetTickCountFromISR();

    portENTER_CRITICAL_ISR(&s_lock);
    switch (f.header.id) {
    case ID_RPM:       /* bytes 2-3, little endian, 0.25 rpm/bit */
        s_rpm = ((d[3] << 8) | d[2]) * 0.25f;
        s_t_rpm = now; s_seen_rpm = true;
        break;
    case ID_COOLANT:   /* byte 1, 0.75 °C/bit, -48 */
        s_coolant = d[1] * 0.75f - 48.0f;
        s_t_coolant = now; s_seen_coolant = true;
        break;
    case ID_INTAKE:    /* byte 1, 0.75 °C/bit, -48 */
        s_intake = d[1] * 0.75f - 48.0f;
        s_t_intake = now; s_seen_intake = true;
        break;
    default:
        break;
    }
    portEXIT_CRITICAL_ISR(&s_lock);
    return false;
}

void can_start(void)
{
    twai_onchip_node_config_t cfg = {
        .io_cfg = {
            .tx = CAN_TX_GPIO,
            .rx = CAN_RX_GPIO,
            .quanta_clk_out = -1,
            .bus_off_indicator = -1,
        },
        .bit_timing = { .bitrate = 500000 },
        .tx_queue_depth = 1,
        .flags = { .enable_listen_only = true },
    };
    twai_node_handle_t node;
    ESP_ERROR_CHECK(twai_new_node_onchip(&cfg, &node));

    twai_event_callbacks_t cbs = { .on_rx_done = on_rx };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(node, &cbs, NULL));
    ESP_ERROR_CHECK(twai_node_enable(node));
    ESP_LOGI(TAG, "CAN listen-only, 500k, TX=%d RX=%d", CAN_TX_GPIO, CAN_RX_GPIO);
}

static bool fresh(bool seen, TickType_t t, TickType_t now)
{
    return seen && (now - t) < pdMS_TO_TICKS(STALE_MS);
}

car_data_t can_get(void)
{
    car_data_t c;
    TickType_t now = xTaskGetTickCount();
    portENTER_CRITICAL(&s_lock);
    c.rpm = s_rpm;
    c.coolant_c = s_coolant;
    c.intake_c = s_intake;
    c.rpm_ok = fresh(s_seen_rpm, s_t_rpm, now);
    c.coolant_ok = fresh(s_seen_coolant, s_t_coolant, now);
    c.intake_ok = fresh(s_seen_intake, s_t_intake, now);
    portEXIT_CRITICAL(&s_lock);
    return c;
}
