/******************************************************************************
 * ESP32 Daikin P1P2 Serial
 * 
 * Copyright (C) 2024  Alexander Hoet <alexanderhoet@gmail.com>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * @file    p1p2_serial.c
 * @author  Alexander Hoet
 * @brief
 * 
 ******************************************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/gptimer.h>
#include <hal/clk_tree_hal.h>

#include "p1p2_serial.h"

#define RX_QUEUE_SIZE           P1P2_MESSAGE_SIZE_MAX
#define TX_QUEUE_SIZE           P1P2_MESSAGE_SIZE_MAX
#define PRX_QUEUE_SIZE          1
#define PTX_QUEUE_SIZE          1
#define RECEIVE_TASK_NAME       "P1P2 Receive"
#define TRANSMIT_TASK_NAME      "P1P2 Transmit"
#define TASK_STACK              2048
#define TASK_PRIORITY           12
#define EOP_CHAR                UINT32_MAX
#define EOP_TIMEOUT             22

#define CheckESPErrorOrExit(x) \
  err = convert_esp_err(x); \
  if(err != P1P2_OK) goto exit;

typedef enum {
  idle,
  receiving,
  transmitting,
  wait_eop
} serial_state_t;

static QueueHandle_t rxqueue = NULL;
static QueueHandle_t txqueue = NULL;
static QueueHandle_t prxqueue = NULL;
static QueueHandle_t ptxqueue = NULL;
static TaskHandle_t serial_receive_task = NULL;
static TaskHandle_t serial_transmit_task = NULL;
static gpio_num_t hb_rx_pin = GPIO_NUM_NC;
static gpio_num_t hb_tx_pin = GPIO_NUM_NC;
static gpio_num_t hb_rst_pin = GPIO_NUM_NC;
static gptimer_handle_t baud_timer = NULL;
static uint32_t baud_timeout = 0;
static serial_state_t serial_state = idle;

static void p1p2_serial_receive_task(void *argument);
static void p1p2_serial_transmit_task(void *argument);
static void byte_encode(uint8_t input, uint32_t *output);
static P1P2_MESSAGE_ERROR byte_decode(uint32_t input, uint8_t *output);
static void crc_accumulate(uint8_t* crc, uint8_t byte);
static void hb_rx_handler(void *argument);
static bool baud_timer_handler(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx);
static p1p2_err_t convert_esp_err(esp_err_t esp_err);

p1p2_err_t p1p2_serial_init(int8_t rx_pin, int8_t tx_pin, int8_t rst_pin)
{
  p1p2_err_t err = P1P2_OK;

  /* Reset serial state to idle */
  serial_state = idle;

  /* Check and save pin mapping */
  if(!GPIO_IS_VALID_GPIO(rx_pin) || !GPIO_IS_VALID_GPIO(tx_pin) || !GPIO_IS_VALID_GPIO(rst_pin)) {
    err =  P1P2_ERR_INVALID_ARG;
    goto exit;
  }

  hb_rx_pin = rx_pin;
  hb_tx_pin = tx_pin;
  hb_rst_pin = rst_pin;

  /* Create the receive and transmit queues */
  rxqueue = xQueueCreate(RX_QUEUE_SIZE, sizeof(uint32_t));
  txqueue = xQueueCreate(TX_QUEUE_SIZE, sizeof(uint32_t));
  prxqueue = xQueueCreate(PRX_QUEUE_SIZE, sizeof(P1P2_Message_t));
  ptxqueue = xQueueCreate(PTX_QUEUE_SIZE, sizeof(P1P2_Message_t));

  if((rxqueue == NULL) || (txqueue == NULL) || (prxqueue == NULL) || (ptxqueue == NULL)) {
    err = P1P2_ERR_NO_MEM;
    goto exit;
  }

  /* Create the p1p2 serial receive task */
  if(xTaskCreate(p1p2_serial_receive_task, RECEIVE_TASK_NAME, TASK_STACK, NULL, TASK_PRIORITY, &serial_receive_task) != pdPASS) {
    err = P1P2_ERR_NO_MEM;
    goto exit;
  }

  /* Create the p1p2 serial transmit task */
  if(xTaskCreate(p1p2_serial_transmit_task, TRANSMIT_TASK_NAME, TASK_STACK, NULL, TASK_PRIORITY, &serial_transmit_task) != pdPASS) {
    err = P1P2_ERR_NO_MEM;
    goto exit;
  }

  /* Install the GPIO driver's ISR handler service */
  CheckESPErrorOrExit(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL2 | ESP_INTR_FLAG_IRAM));

  /* Configure Home Bus RX pin */
  const gpio_config_t gpio_config_rx = {
    .pin_bit_mask = BIT64(hb_rx_pin),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = false,
    .pull_down_en = false,
    .intr_type = GPIO_INTR_NEGEDGE,
  };

  CheckESPErrorOrExit(gpio_config(&gpio_config_rx));

  /* Configure Home Bus TX and RST pin */
  const gpio_config_t gpio_config_tx_rst = {
    .pin_bit_mask = BIT64(hb_tx_pin) | BIT64(hb_rst_pin),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = false,
    .pull_down_en = false,
    .intr_type = GPIO_INTR_DISABLE,
  };

  CheckESPErrorOrExit(gpio_set_level(hb_tx_pin, 1));
  CheckESPErrorOrExit(gpio_set_level(hb_rst_pin, 1));
  CheckESPErrorOrExit(gpio_config(&gpio_config_tx_rst));

  /* Create the baud rate timer, run the timer as fast as possible */
  const gptimer_config_t baud_timer_config = {
    .clk_src = GPTIMER_CLK_SRC_APB,
    .direction = GPTIMER_COUNT_UP, 
    .resolution_hz = clk_hal_apb_get_freq_hz() >> 1,
    .flags.intr_shared = 0};
  
  CheckESPErrorOrExit(gptimer_new_timer(&baud_timer_config, &baud_timer));
  
  /* Calculate the baud timeout */
  CheckESPErrorOrExit(gptimer_get_resolution(baud_timer, &baud_timeout));
  baud_timeout /= (P1P2_BAUD_RATE * 2);

  /* Set the alarm count to the baud rate */
  const gptimer_alarm_config_t baud_timer_alarm_config = {
    .alarm_count = baud_timeout,
    .reload_count = 0,
    .flags.auto_reload_on_alarm = true};

  CheckESPErrorOrExit(gptimer_set_alarm_action(baud_timer, &baud_timer_alarm_config));

  /* Register the baud timer rate event callback */
  const gptimer_event_callbacks_t baud_timer_event_callbacks = {
    .on_alarm = baud_timer_handler};

  CheckESPErrorOrExit(gptimer_register_event_callbacks(baud_timer, &baud_timer_event_callbacks, NULL));

  /* Enable and start the baud timer */
  CheckESPErrorOrExit(gptimer_enable(baud_timer));
  CheckESPErrorOrExit(gptimer_start(baud_timer));

  /* Enable the Home Bus RX pin interrupt */
  CheckESPErrorOrExit(gpio_isr_handler_add(hb_rx_pin, hb_rx_handler, NULL));

  return P1P2_OK;

exit:
  p1p2_serial_deinit();
  return err;
}

p1p2_err_t p1p2_serial_deinit(void)
{
  /* Disable the Home Bus RX pin interrupt */
  gpio_intr_disable(hb_rx_pin);

  /* Disable and delete the baud timer */
  gptimer_disable(baud_timer);
  gptimer_del_timer(baud_timer);

  /* Reset all gpio pins */
  gpio_reset_pin(hb_rx_pin);
  gpio_reset_pin(hb_tx_pin);
  gpio_reset_pin(hb_rst_pin);

  /* Delete the Serial Tasks */
  vTaskDelete(serial_receive_task);
  vTaskDelete(serial_transmit_task);

  /* Delete all receive and transmit queues */
  vQueueDelete(rxqueue);
  vQueueDelete(txqueue);
  vQueueDelete(prxqueue);
  vQueueDelete(ptxqueue);

  return P1P2_OK;
}

p1p2_err_t p1p2_message_read(P1P2_Message_t *message, uint32_t timeout)
{
  if(prxqueue == NULL)
    return P1P2_ERR_INVALID_STATE;

  if(message == NULL)
    return P1P2_ERR_INVALID_ARG;

  if(xQueueReceive(prxqueue, message, timeout) != pdPASS)
    return P1P2_ERR_TIMEOUT;

  return P1P2_OK;
}

p1p2_err_t p1p2_message_write(P1P2_Message_t *message, uint32_t timeout)
{
  if(ptxqueue == NULL)
    return P1P2_ERR_INVALID_STATE;

  if(message == NULL)
    return P1P2_ERR_INVALID_ARG;

  if(xQueueSendToBack(ptxqueue, message, timeout) != pdPASS)
    return P1P2_ERR_TIMEOUT;

  return P1P2_OK;
}

static void p1p2_serial_receive_task(void *argument)
{
  uint32_t input = 0;
  uint8_t rx_size = 0;
  uint8_t rx_data[P1P2_MESSAGE_SIZE_MAX];
  P1P2_Message_t rxmessage = {0};

  for(;;) {
    /* Get a new byte off the receive queue */
    if(xQueueReceive(rxqueue, &input, portMAX_DELAY) == pdPASS) {
      /* Check if the received byte indicates a message timeout */
      if(input != EOP_CHAR) {
        /* Process the received data byte and add it to the end of the current message */
        if(rx_size < P1P2_MESSAGE_SIZE_MAX)
          rxmessage.error_flags |= byte_decode(input, &rx_data[rx_size++]);
        else
          ESP_LOGD("P1P2SERIAL", "Oversized message");
      } else {
        uint8_t crc = 0;
        
        if(rx_size < P1P2_MESSAGE_SIZE_MIN) {
          ESP_LOGD("P1P2SERIAL", "Undersized message");
          rx_size = 0;
          continue;
        }

        for(uint8_t i = 0; i < rx_size - 1; i++)
            crc_accumulate(&crc, rx_data[i]);

        if(crc != rx_data[rx_size - 1]) {
          ESP_LOGD("P1P2SERIAL", "CRC Error %02x, %02x", crc, rx_data[rx_size - 1]);
          rxmessage.error_flags |= P1P2_MESSAGE_ERROR_CRC;
        }

        rxmessage.direction = rx_data[0];
        rxmessage.deviceid = rx_data[1];
        rxmessage.messageid = rx_data[2];

        rxmessage.datasize = 0;

        for(uint8_t i = 3; i < rx_size - 1; i++)
          rxmessage.data[rxmessage.datasize++] = rx_data[i];

        /* Push the message to the message queue */
        if(xQueueSendToBack(prxqueue, &rxmessage, 1) != pdPASS)
          ESP_LOGD("P1P2SERIAL", "Receive buffer overflow");

        rx_size = 0;
        memset(&rxmessage, 0, sizeof(P1P2_Message_t));
      }
    }
  }
}

static void p1p2_serial_transmit_task(void *argument)
{
  P1P2_Message_t txmessage = {0};

  for(;;) {
    /* Get a new message of the queue */
    if(xQueueReceive(ptxqueue, &txmessage, portMAX_DELAY) == pdPASS) {
      uint8_t crc = 0;
      uint32_t output = 0;
      uint8_t tx_size = 0;
      uint8_t tx_data[P1P2_MESSAGE_SIZE_MAX] = {0};

      /* Copy message data to transmit buffer */
      tx_data[0] = txmessage.direction;
      tx_data[1] = txmessage.deviceid;
      tx_data[2] = txmessage.messageid;
      tx_size = 3;

      for(uint8_t i = 0; i < txmessage.datasize; i++)
          tx_data[tx_size++] = txmessage.data[i];

      /* Calculate crc and add to end of transmit buffer */
      for(uint8_t i = 0; i < tx_size; i++)
            crc_accumulate(&crc, tx_data[i]);

      tx_data[tx_size++] = crc;

      /* Encode and transmit all bytes from transmit buffer */
      for(uint8_t i = 0; i < tx_size; i++) {
        byte_encode(tx_data[i], &output);

        if(xQueueSendToBack(txqueue, &output, 0) != pdPASS) {
          ESP_LOGD("P1P2SERIAL", "Transmit buffer overflow");
          continue;
        }
      }
    }
  }
}

static void byte_encode(uint8_t input, uint32_t *output)
{
	uint8_t parity = 0;

  *output = 0;

	/* Put data bits into output */
	for (uint8_t i = 0; i < 8; i++) {
		if (input & 0x1) {
			parity++;
			*output |= 0x1;
		}

		*output <<= 1;
		input >>= 1;
	}

	/* Set the parity bit */
	if (parity & 0x1) {
		*output |= 0x1;
	}

	/* Set the stop bit */
	*output <<= 1;
	*output |= 0x1;
}

static P1P2_MESSAGE_ERROR byte_decode(uint32_t input, uint8_t *output)
{
  uint8_t parity = 0;
  uint8_t error_flags = P1P2_MESSAGE_ERROR_NONE;

  *output = 0;

  /* First bit must be zero to indicate start condition */
  if((input & 0x00200000) != 0) {
    error_flags |= P1P2_MESSAGE_ERROR_START_BIT;
    ESP_LOGD("P1P2SERIAL", "Start Bit Error %08lx", input);
  }

  /* Second last bit must always be one to indicate stop condition */
  if((input & 0x00000002) != 0x00000002) {
    error_flags |= P1P2_MESSAGE_ERROR_STOP_BIT;
    ESP_LOGD("P1P2SERIAL", "Stop Bit Error %08lx", input);
  }

  /* Even bits must always be one */
  if((input & 0x00155555) != 0x00155555) { 
    error_flags |= P1P2_MESSAGE_ERROR_FRAMING;
    ESP_LOGD("P1P2SERIAL", "Framing Error %08lx", input);
  }

  /* Shift data bytes into output */
  for(uint8_t i = 5; i <= 19; i += 2) {
    *output <<= 1;

    if((input & (1 << i)) != 0) {
      parity++;
      *output |= 0x01;
    }
  }

  /* Check parity */
  if(((input >> 3) & 0x1) != (parity & 0x1)) {
    error_flags |= P1P2_MESSAGE_ERROR_PARITY;
    ESP_LOGD("P1P2SERIAL", "Parity Error %08lx", input);
  }

  return error_flags;
}

static void crc_accumulate(uint8_t* crc, uint8_t byte)
{
	for (uint8_t i = 0; i < 8; i++) {
	  *crc = (((*crc ^ byte) & 0x01) ? ((*crc >> 1) ^ 0xD9) : (*crc >> 1));
	  byte >>= 1;
	}
}

static void IRAM_ATTR hb_rx_handler(void *argument)
{  
  /* Do not sync timer when we are transmitting */
  if(serial_state == transmitting)
    return;

  /* Set the baud timer counter to halfway the baud timeout value
  * This will allow us the trigger the alarm halfway the data pulse */
  gptimer_set_raw_count(baud_timer, (baud_timeout >> 1));
}

static bool IRAM_ATTR baud_timer_handler(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
  static uint8_t bitcounter = 0;
  static uint32_t inputbuffer = 0;
  static uint32_t outputbuffer = 0;

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  /* Shift the received bit into the buffer */
  inputbuffer <<= 1;
  inputbuffer |= (gpio_get_level(hb_rx_pin) & 0x01);

  switch (serial_state)
  {
  case idle:
    if((inputbuffer & 0x3) == 0x1) {
      bitcounter = 2;
      serial_state = receiving;
    } else if(xQueuePeekFromISR(txqueue, &outputbuffer) == pdTRUE) {
      bitcounter = 0;
      serial_state = transmitting;
    }
    break;
  case receiving:
    bitcounter++;
    /* If 22 bits have been received */
    if (bitcounter == 22) {
      /* Put the received data into the queue */
      xQueueSendToBackFromISR(rxqueue, &inputbuffer, &xHigherPriorityTaskWoken);

      /* Reset the counter and buffer */
      bitcounter = 0;

      /* Wait for timeout */
      serial_state = wait_eop;
    }
    
    break;
  case transmitting:
    /* If the bitcounter is zero try to fetch a new character from the queue */
    if (bitcounter == 0) {
      if(xQueueReceiveFromISR(txqueue, &outputbuffer, &xHigherPriorityTaskWoken) == pdTRUE)
        bitcounter = 22; 
    }
     
    if (bitcounter == 0) {
      /* Go to idle */
      serial_state = idle;
    } else {
      /* Shift the received bit into the receivebuffer */
      gpio_set_level(hb_tx_pin, ((outputbuffer >> 22) & 0x01));
      outputbuffer <<= 1;

      /* Decrement the bitcounter */
      bitcounter--;
    }
    break;
  case wait_eop:
    /* Check for retransmit */
    if((inputbuffer & 0x3) == 0x1) {
      bitcounter = 2;
      serial_state = receiving;
    } else {
      bitcounter++;

      if(bitcounter == EOP_TIMEOUT) {
        /* Put the eop character into the queue */
        uint32_t tmp = EOP_CHAR;
        xQueueSendToBackFromISR(rxqueue, &tmp, &xHigherPriorityTaskWoken);

        /* Reset the counter and buffer */
        bitcounter = 0;

        /* Go to idle */
        serial_state = idle;
      }
    }
    
    break;
  }

  return (xHigherPriorityTaskWoken == pdTRUE);
}

static p1p2_err_t convert_esp_err(esp_err_t esp_err)
{
  switch (esp_err) {
  case ESP_OK:
    return P1P2_OK;
  case ESP_ERR_NO_MEM:
    return P1P2_ERR_NO_MEM;
  case ESP_ERR_INVALID_ARG:
    return P1P2_ERR_INVALID_ARG;
  case ESP_ERR_INVALID_STATE:
    return P1P2_ERR_INVALID_STATE;
  case ESP_ERR_NOT_FOUND:
    return P1P2_ERR_NOT_FOUND;
  default:
    return P1P2_ERR_FAIL;
  }
}