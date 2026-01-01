/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

#define TWAI_SENDER_TX_GPIO     GPIO_NUM_16
#define TWAI_SENDER_RX_GPIO     GPIO_NUM_17
#define TWAI_QUEUE_DEPTH        10
#define TWAI_BITRATE            1000000

// Message IDs
#define TWAI_DATA_ID            0x100
#define TWAI_HEARTBEAT_ID       0x7FF
#define TWAI_DATA_LEN           1000

static const char *TAG = "twai_sender";

static QueueHandle_t rx_queue = NULL;

typedef struct {
    twai_frame_t frame;
    uint8_t data[TWAI_FRAME_MAX_LEN];
} twai_data_t;


// Transmission completion callback
static IRAM_ATTR bool twai_sender_tx_done_callback(twai_node_handle_t handle, const twai_tx_done_event_data_t *edata, void *user_ctx)
{
    if (!edata->is_tx_success) {
        ESP_EARLY_LOGW(TAG, "Failed to transmit message, ID: 0x%X", edata->done_tx_frame->header.id);
    }
    return false; // No task wake required
}

// Bus error callback
static IRAM_ATTR bool twai_sender_on_error_callback(twai_node_handle_t handle, const twai_error_event_data_t *edata, void *user_ctx)
{
    ESP_EARLY_LOGW(TAG, "TWAI node error: 0x%x", edata->err_flags.val);
    return false; // No task wake required
}

// TWAI receive callback - store data and signal
static bool IRAM_ATTR twai_listener_rx_callback(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    // Callback ISR (Interrupt Service Routine) -> ESP32 si interrompe perché è arrivato un messaggio
    // Serve gestire l'interrupt nel minor tempo possibile (microsecondi)
    // Quindi mettiamo il messaggio in una coda, così un task specifico lo potrà gestire
    uint8_t recv_buff[8];
    twai_frame_t frame = {
        .buffer = recv_buff,
        .buffer_len = sizeof(recv_buff),
    };
    BaseType_t high_task_wakeup = pdFALSE;

    if (twai_node_receive_from_isr(handle, &frame) == ESP_OK) {
        // Invia alla coda. Se è piena, il messaggio viene ignorato
        xQueueSendFromISR(rx_queue, &frame, &high_task_wakeup);
    }
    return (high_task_wakeup == pdTRUE);
}

// Task di elaborazione della coda dei messaggi CAN ricevuti
void twai_log_task(void *pvParameters)
{
    twai_frame_t rx_msg;
    ESP_LOGI(TAG, "Task di logging avviato e pronto.");

    while (1) {
        // Il task rimane "Sospeso" qui finché non c'è un dato nella coda.
        // Non consuma CPU mentre aspetta.
        if (xQueueReceive(rx_queue, &rx_msg, portMAX_DELAY) == pdTRUE) {
            
            ESP_LOGI(TAG, "RX: %x [%d] %x %x %x %x %x %x %x %x", \
              rx_msg.header.id, rx_msg.header.dlc, rx_msg.buffer[0], rx_msg.buffer[1], rx_msg.buffer[2], rx_msg.buffer[3], rx_msg.buffer[4], rx_msg.buffer[5], rx_msg.buffer[6], rx_msg.buffer[7]);
        }
    }
}

static bool IRAM_ATTR twai_listener_on_state_change_callback(twai_node_handle_t handle, const twai_state_change_event_data_t *edata, void *user_ctx)
{
    const char *twai_state_name[] = {"error_active", "error_warning", "error_passive", "bus_off"};
    ESP_EARLY_LOGI(TAG, "state changed: %s -> %s", twai_state_name[edata->old_sta], twai_state_name[edata->new_sta]);
    return false;
}

extern "C" void app_main(void)
{
    printf("===================TWAI Sender Example Starting...===================\n");
    twai_node_handle_t sender_node = NULL;
    rx_queue = xQueueCreate(20, sizeof(twai_frame_t));

    // Configure TWAI node
    twai_onchip_node_config_t node_config = {
        .io_cfg = {
            .tx = TWAI_SENDER_TX_GPIO,
            .rx = TWAI_SENDER_RX_GPIO,
            // .quanta_clk_out = -1,
            // .bus_off_indicator = -1,
        },
        .bit_timing = {
            .bitrate = TWAI_BITRATE,
        },
        .fail_retry_cnt = 3,
        .tx_queue_depth = TWAI_QUEUE_DEPTH,
        .flags {
            .enable_self_test = 1,
            .enable_loopback = 1, // Necessario per ricevere i messaggi inviati da me steso e poter usare la modalità self_test 
            // .enable_listen_only = 1 // Imposta TWAI in sola lettura
        }
    };

    // Create TWAI node
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &sender_node));

    // Register transmission completion callback
    twai_event_callbacks_t callbacks = {
        //.on_tx_done = twai_sender_tx_done_callback,
        .on_rx_done = twai_listener_rx_callback,
        .on_state_change = twai_listener_on_state_change_callback,
        .on_error = twai_sender_on_error_callback,
    };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(sender_node, &callbacks, NULL));

    xTaskCreate(twai_log_task, "twai_log_task", 4096, NULL, 5, NULL);

    // Enable TWAI node
    ESP_ERROR_CHECK(twai_node_enable(sender_node));
    ESP_LOGI(TAG, "TWAI Sender started successfully");
    ESP_LOGI(TAG, "Sending messages with IDs: 0x%03X (data), 0x%03X (heartbeat)",  TWAI_DATA_ID, TWAI_HEARTBEAT_ID);

    while (1) {
        // Send heartbeat message
        uint64_t timestamp = esp_timer_get_time();
        twai_frame_t tx_frame = {
            .header =
                { .id = TWAI_HEARTBEAT_ID },
            .buffer = (uint8_t *) &timestamp,
            .buffer_len = sizeof(timestamp),
        };
        ESP_ERROR_CHECK(twai_node_transmit(sender_node, &tx_frame, 500));
        ESP_LOGI(TAG, "Sending heartbeat message: %lld", timestamp);
        ESP_ERROR_CHECK(twai_node_transmit_wait_all_done(sender_node, -1)); // -1 means wait forever

        // Send burst data messages every 10 seconds
        
        if ((timestamp / 1000000) % 10 == 0) {
            int num_frames = howmany(TWAI_DATA_LEN, TWAI_FRAME_MAX_LEN);
            twai_data_t *data = (twai_data_t *)calloc(num_frames, sizeof(twai_data_t));
            assert(data != NULL);
            ESP_LOGI(TAG, "Sending packet of %d bytes in %d frames", TWAI_DATA_LEN, num_frames);
            for (int i = 0; i < num_frames; i++) {
                data[i].frame.header.id = TWAI_DATA_ID;
                data[i].frame.buffer = data[i].data;
                data[i].frame.buffer_len = TWAI_FRAME_MAX_LEN;
                memset(data[i].data, i, TWAI_FRAME_MAX_LEN);
                ESP_ERROR_CHECK(twai_node_transmit(sender_node, &data[i].frame, 500));
            }

            // Frames mounted, wait for all frames to be transmitted
            ESP_ERROR_CHECK(twai_node_transmit_wait_all_done(sender_node, -1));
            free(data);
        }
        

        vTaskDelay(pdMS_TO_TICKS(1000));
        twai_node_status_t status;
        twai_node_get_info(sender_node, &status, NULL);
        if (status.state == TWAI_ERROR_BUS_OFF) {
            ESP_LOGW(TAG, "Bus-off detected");
            return;
        }
    }

    ESP_ERROR_CHECK(twai_node_disable(sender_node));
    ESP_ERROR_CHECK(twai_node_delete(sender_node));
}
