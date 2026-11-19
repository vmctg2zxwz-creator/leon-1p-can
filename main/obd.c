#include "obd.h"
#include <string.h>
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

/*
 * Leon 1P (PQ35) powertrain CAN, 500 kbit/s, ΜΟΝΟ ακρόαση (listen-only):
 * δεν στέλνουμε τίποτα στο αυτοκίνητο.
 * Pins όπως στο waveshare-dash-firmware (KIT-B): TX GPIO2, RX GPIO3.
 */
#define CAN_TX_GPIO 2
#define CAN_RX_GPIO 3
#define STALE_MS    1000

/* Broadcast IDs (επιβεβαιωμένα σε Autel για τα 4 πρώτα). */
#define ID_MOTOR_1  0x280  /* στροφές */
#define ID_BREMSE_1 0x1A0  /* ταχύτητα ABS */
#define ID_MOTOR_2  0x288  /* νερό */
#define ID_MOTOR_3  0x380  /* αέρας εισαγωγής */

static const char *TAG = "can";
static car_data_t s_data;
static TickType_t s_last_rx;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

/* ΠΡΟΣΟΧΗ: οι κλίμακες είναι οι συνήθεις για PQ35 και θέλουν σύγκριση με Autel. */
static void decode(const twai_message_t *m)
{
    const uint8_t *d = m->data;
    portENTER_CRITICAL(&s_lock);
    switch (m->identifier) {
    case ID_MOTOR_1:
        if (m->data_length_code >= 4) s_data.rpm = ((d[3] << 8) | d[2]) * 0.25f;
        break;
    case ID_BREMSE_1:
        if (m->data_length_code >= 4) s_data.speed_kmh = (((d[3] << 8) | d[2]) >> 1) * 0.01f;
        break;
    case ID_MOTOR_2:
        if (m->data_length_code >= 2) s_data.coolant_c = d[1] * 0.75f - 48.0f;
        break;
    case ID_MOTOR_3:
        if (m->data_length_code >= 2) s_data.intake_c = d[1] * 0.75f - 48.0f;
        break;
    default:
        portEXIT_CRITICAL(&s_lock);
        return;
    }
    s_last_rx = xTaskGetTickCount();
    s_data.connected = true;
    portEXIT_CRITICAL(&s_lock);
}

static void can_task(void *arg)
{
    twai_message_t rx;
    for (;;) {
        if (twai_receive(&rx, pdMS_TO_TICKS(100)) == ESP_OK && !(rx.flags & TWAI_MSG_FLAG_EXTD)) {
            decode(&rx);
        }

        portENTER_CRITICAL(&s_lock);
        if (s_data.connected && (xTaskGetTickCount() - s_last_rx) > pdMS_TO_TICKS(STALE_MS)) {
            s_data.connected = false;
        }
        portEXIT_CRITICAL(&s_lock);

        twai_status_info_t st;
        if (twai_get_status_info(&st) == ESP_OK && st.state == TWAI_STATE_BUS_OFF) {
            twai_initiate_recovery();
            vTaskDelay(pdMS_TO_TICKS(500));
            twai_start();
        }
    }
}

void obd_start(void)
{
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_LISTEN_ONLY);
    g.rx_queue_len = 64;
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g, &t, &f));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "CAN listen-only (TX=%d RX=%d, 500k)", CAN_TX_GPIO, CAN_RX_GPIO);

    xTaskCreate(can_task, "can", 4096, NULL, 5, NULL);
}

car_data_t obd_get(void)
{
    car_data_t copy;
    portENTER_CRITICAL(&s_lock);
    copy = s_data;
    portEXIT_CRITICAL(&s_lock);
    return copy;
}
