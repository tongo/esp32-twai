#pragma once

#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define RAW_MESSAGES_QUEUE_SIZE 20
#define TWAI_TX_GPIO GPIO_NUM_16 // GPIO_NUM_NC | GPIO_NUM_16
#define TWAI_RX_GPIO GPIO_NUM_17
#define TWAI_QUEUE_DEPTH        10
#define TWAI_BITRATE            125000

// TEST Message IDs
#define TWAI_DATA_ID            0x100
#define TWAI_HEARTBEAT_ID       0x7FF
#define TWAI_DATA_LEN           1000

typedef struct {
    twai_frame_t frame;
    uint8_t data[TWAI_FRAME_MAX_LEN];
} twai_data_t;

class CanManager {
public:
    twai_node_handle_t twaiNodeHandle;
    QueueHandle_t rawCanMessagesQueue;

    CanManager(bool selfTestMode = false);
    ~CanManager();
    void begin();
    void enableNode();
    void disableNode();
private:
    static constexpr const char* TAG = "can_manager";
    bool _isSelfTestMode;
    
    TaskHandle_t _ellaborazioneTaskHandle;
    TaskHandle_t _simulazioneTaskHandle;

    // TWAI callbacks
    static bool IRAM_ATTR twai_rx_callback(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx);

    static void taskElaborazioneMessaggiCan(void* pvParameters);
    static void taskSimulazioneMessaggiCan(void* pvParameters);
};