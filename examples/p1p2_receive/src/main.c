#include <esp_log.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <p1p2_serial.h>

#define HB_RX_PIN               1
#define HB_TX_PIN               0
#define HB_RST_PIN              3

static void message_print(P1P2_Message_t *message);

void app_main() 
{
    P1P2_Message_t rxmessage = {0};

    esp_log_level_set("P1P2SERIAL", ESP_LOG_DEBUG);

    if (p1p2_serial_init(HB_RX_PIN, HB_TX_PIN, HB_RST_PIN) != P1P2_OK)
    {
        p1p2_serial_deinit();
        return;
    }

    for (;;)
    {
        if(p1p2_message_read(&rxmessage, portMAX_DELAY) == P1P2_OK) {
            message_print(&rxmessage);
        } else {
            vTaskDelay(1);
        }
    }
}

static void message_print(P1P2_Message_t *message)
{
	char buffer[100] = {0};
    char* bufferptr = buffer;
    
    sprintf(bufferptr, "%02x %02x %02x: ", message->direction, message->deviceid, message->messageid);
    bufferptr += 10;

    for (uint8_t i = 0; i < message->datasize; i++) {
		sprintf(bufferptr, "%02x ", message->data[i]);
        bufferptr += 3;
	}

    ESP_LOGI("P1P2", "%s", buffer);
}