#include "can_manager.hpp"
#include <sys/param.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"

CanManager::CanManager(bool selfTestMode) {
    _isSelfTestMode = selfTestMode;
}

CanManager::~CanManager() {
    disableNode();
    ESP_ERROR_CHECK(twai_node_delete(twaiNodeHandle));
    twaiNodeHandle = NULL;
    rawCanMessagesQueue = NULL;

    _ellaborazioneTaskHandle = NULL;
    _simulazioneTaskHandle = NULL;
}

// Inizializza il Node TWAI e le callback
void CanManager::begin() {
    twai_onchip_node_config_t nodeConfig = {
        .io_cfg = {
            .tx = TWAI_TX_GPIO,
            .rx = TWAI_RX_GPIO,
            // .quanta_clk_out = -1,
            // .bus_off_indicator = -1,
        },
        .bit_timing = {
            .bitrate = TWAI_BITRATE,
        },
        .fail_retry_cnt = 3,
        .tx_queue_depth = TWAI_QUEUE_DEPTH,
    };
    if (_isSelfTestMode) {
        nodeConfig.flags = {
            .enable_self_test = 1,
            .enable_loopback = 1, // Necessario per ricevere i messaggi inviati da me steso e poter usare la modalità self_test 
        };
    } else {
        nodeConfig.flags = {
            .enable_listen_only = 1
        };
    }

    // Create TWAI node
    ESP_ERROR_CHECK(twai_new_node_onchip(&nodeConfig, &twaiNodeHandle));

    // Register transmission completion callback
    twai_event_callbacks_t callbacks = {
        //.on_tx_done = twai_sender_tx_done_callback,
        .on_rx_done = twai_rx_callback,
        // .on_state_change = twai_listener_on_state_change_callback,
        // .on_error = twai_sender_on_error_callback,
    };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(twaiNodeHandle, &callbacks, this));
}

// Abilita il node TWAI per la ricezione dei messaggi (e l'invio dei dati simulati in caso di self_test)
void CanManager::enableNode() {
    // Creazione della coda dei messaggi
    rawCanMessagesQueue = xQueueCreate(RAW_MESSAGES_QUEUE_SIZE, sizeof(twai_frame_t));

    // Crea il task di elaborazione della coda e lo fa partire
    BaseType_t ret = xTaskCreatePinnedToCore(
        taskElaborazioneMessaggiCan,   // Funzione del task
        "task_elaborazione_can",         // Nome testuale
        4096,                   // Stack size
        this,                   // Parametro passato (l'istanza della classe)
        5,                      // Priorità task
        &_ellaborazioneTaskHandle, // INDIRIZZO DELL'HANDLE (Fondamentale!)
        1                       // Core affinity
    );

    ESP_ERROR_CHECK(twai_node_enable(twaiNodeHandle));

    if (_isSelfTestMode) {
        // Task di generazione dei messaggi simulati per il self_test
        BaseType_t ret = xTaskCreatePinnedToCore(
            taskSimulazioneMessaggiCan,   // Funzione del task
            "task_simulazione_can",         // Nome testuale
            4096,                   // Stack size
            this,                   // Parametro passato (l'istanza della classe)
            5,                      // Priorità task
            &_simulazioneTaskHandle, // INDIRIZZO DELL'HANDLE (Fondamentale!)
            1                       // Core affinity
        );
    }
}

// Disabilita il node TWAI e ferma i task di elaborazione dati
void CanManager::disableNode() {
    if (_isSelfTestMode && _simulazioneTaskHandle != NULL) {
        vTaskDelete(_simulazioneTaskHandle);
        _simulazioneTaskHandle = NULL;
    }
    ESP_ERROR_CHECK(twai_node_disable(twaiNodeHandle));

    vTaskDelete(_ellaborazioneTaskHandle);
    _ellaborazioneTaskHandle = NULL;

    if (rawCanMessagesQueue != NULL) {
        vQueueDelete(rawCanMessagesQueue);
    }
}

// Task che elabora la coda dei RAW Messages CAN
void CanManager::taskElaborazioneMessaggiCan(void *pvParameters) {
    CanManager* instance = static_cast<CanManager*>(pvParameters);
    twai_frame_t rx_msg;
    ESP_LOGI(TAG, "Task di elaborazione Messaggi CAN avviato.");

    while (1) {
        // Il task rimane "Sospeso" qui finché non c'è un dato nella coda.
        // Non consuma CPU mentre aspetta.
        if (xQueueReceive(instance->rawCanMessagesQueue, &rx_msg, portMAX_DELAY) == pdTRUE) {
            // printf("%d,%x,%x,%x,%x,%x,%x,%x,%x", rx_msg.header.id, rx_msg.buffer[0], rx_msg.buffer[1], rx_msg.buffer[2], rx_msg.buffer[3], rx_msg.buffer[4], rx_msg.buffer[5], rx_msg.buffer[6], rx_msg.buffer[7]);
            
            ESP_LOGI(TAG, "%x,%x,%x,%x,%x,%x,%x,%x,%x", \
              rx_msg.header.id, rx_msg.buffer[0], rx_msg.buffer[1], rx_msg.buffer[2], rx_msg.buffer[3], rx_msg.buffer[4], rx_msg.buffer[5], rx_msg.buffer[6], rx_msg.buffer[7]);
        }
    }
}

void CanManager::taskSimulazioneMessaggiCan(void *pvParameters) {
    ESP_LOGI(TAG, "Task di simulazione messaggi CAN avviato.");
    CanManager* instance = static_cast<CanManager*>(pvParameters);

    while (1) {
        // Send heartbeat message
        uint64_t timestamp = esp_timer_get_time();
        twai_frame_t tx_frame = {
            .header = { .id = TWAI_HEARTBEAT_ID },
            .buffer = (uint8_t *) &timestamp,
            .buffer_len = sizeof(timestamp),
        };
        ESP_ERROR_CHECK(twai_node_transmit(instance->twaiNodeHandle, &tx_frame, 500));
        ESP_LOGI(TAG, "Sending heartbeat message: %lld", timestamp);
        ESP_ERROR_CHECK(twai_node_transmit_wait_all_done(instance->twaiNodeHandle, -1)); // -1 means wait forever

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
                ESP_ERROR_CHECK(twai_node_transmit(instance->twaiNodeHandle, &data[i].frame, 500));
            }

            // Frames mounted, wait for all frames to be transmitted
            ESP_ERROR_CHECK(twai_node_transmit_wait_all_done(instance->twaiNodeHandle, -1));
            free(data);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
        twai_node_status_t status;
        twai_node_get_info(instance->twaiNodeHandle, &status, NULL);
        if (status.state == TWAI_ERROR_BUS_OFF) {
            ESP_LOGW(TAG, "Bus-off detected");
            return;
        }
    }
}

// TWAI callbacks
bool IRAM_ATTR CanManager::twai_rx_callback(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx) {
    // Callback ISR (Interrupt Service Routine) -> ESP32 si interrompe perché è arrivato un messaggio
    // Serve gestire l'interrupt nel minor tempo possibile (microsecondi)
    // Quindi mettiamo il messaggio in una coda, così un task specifico lo potrà gestire
    CanManager* instance = static_cast<CanManager*>(user_ctx);

    uint8_t recv_buff[8];
    twai_frame_t frame = {
        .buffer = recv_buff,
        .buffer_len = sizeof(recv_buff),
    };
    BaseType_t high_task_wakeup = pdFALSE;

    if (twai_node_receive_from_isr(handle, &frame) == ESP_OK) {
        // Invia alla coda. Se è piena, il messaggio viene ignorato
        xQueueSendFromISR(instance->rawCanMessagesQueue, &frame, &high_task_wakeup);
    }
    return (high_task_wakeup == pdTRUE);
}



/*
// Codice di esempio per elaborare i messaggi CAN
            
switch (message.identifier) {
    case 0x010: { // ID GIRI E TPS
        // Formula Ducati 5AM: RPM = (MSB << 8 | LSB) / 4
        uint16_t raw_rpm = (message.data[0] << 8) | message.data[1];
        float rpm = raw_rpm * 0.25f;
        
        // TPS: Valore grezzo 0-255 (0% - 100%)
        uint8_t tps_raw = message.data[2];
        float tps_perc = (tps_raw / 255.0f) * 100.0f;

        ESP_LOGI(TAG, "MOTORE -> RPM: %.0f | TPS: %.1f%%", rpm, tps_perc);
        break;
    }

    case 0x018: { // ID TEMPERATURE
        // Temperatura Acqua (ECT) con offset standard -40
        int8_t ect = message.data[0] - 40;
        // Temperatura Aria (IAT)
        int8_t iat = message.data[1] - 40;

        ESP_LOGI(TAG, "SENSORI -> H2O: %d°C | Aria: %d°C", ect, iat);
        break;
    }

    case 0x020: { // ID VELOCITÀ
        // Velocità calcolata in km/h
        uint16_t speed = (message.data[0] << 8) | message.data[1];
        ESP_LOGI(TAG, "VELOCITA -> %d km/h", speed);
        break;
    }
}

*/