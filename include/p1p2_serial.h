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
 * @file    p1p2_serial.h
 * @author  Alexander Hoet
 * @brief
 * 
 ******************************************************************************/

#ifndef P1P2_SERIAL_H_
#define P1P2_SERIAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define P1P2_BAUD_RATE               	9600

#define P1P2_MESSAGE_SIZE_MIN   		4
#define P1P2_MESSAGE_SIZE_MAX   		24

#define P1P2_MESSAGE_ERROR_NONE         0
#define P1P2_MESSAGE_ERROR_PARITY       1
#define P1P2_MESSAGE_ERROR_START_BIT    2
#define P1P2_MESSAGE_ERROR_STOP_BIT     4
#define P1P2_MESSAGE_ERROR_FRAMING      8
#define P1P2_MESSAGE_ERROR_CRC          16

typedef uint8_t P1P2_MESSAGE_ERROR;

typedef struct {
    uint8_t direction;
	uint8_t deviceid;
	uint8_t messageid;
	uint8_t data[P1P2_MESSAGE_SIZE_MAX];
	uint8_t datasize;
    P1P2_MESSAGE_ERROR error_flags;
} P1P2_Message_t;

#define P1P2_OK                   0   /*!< esp_err_t value indicating success (no error) */
#define P1P2_ERR_FAIL             1   /*!< Generic esp_err_t code indicating failure */
#define P1P2_ERR_NO_MEM           2   /*!< Out of memory */
#define P1P2_ERR_INVALID_ARG      3   /*!< Invalid argument */
#define P1P2_ERR_INVALID_STATE    4   /*!< Invalid state */
#define P1P2_ERR_NOT_FOUND        5   /*!< Requested resource not found */
#define P1P2_ERR_NOT_SUPPORTED    6   /*!< Operation or feature not supported */
#define P1P2_ERR_TIMEOUT          7   /*!< Operation timed out */

typedef int p1p2_err_t;

/**
 * @brief Initialializes the p1p2 serial component
 * 
 * @note - Creates receive and transmit queues
 * @note - Creates the serial task
 * @note - Configures gpio pins
 * @note - Configures baud rate timer
 * @note - Creates timer and rx pin interrupt handlers
 * 
 * @param[in] rx_pin The pin number of the MAX22088 dout pin
 * @param[in] tx_pin The pin number of the MAX22088 din pin
 * @param[in] rst_pin The pin number of the MAX22088 rst pin
 * @return `p1p2_err_t` error code
 * 
 */
p1p2_err_t p1p2_serial_init(int8_t hb_rx_pin, int8_t hb_tx_pin, int8_t hb_rst_pin);

/**
 * @brief Deinitialializes the p1p2 serial component
 * 
 * @note - Disables timer in rx pin interrupts
 * @note - Deletes baud rate timer
 * @note - Resets gpio pins
 * @note - Deletes serial task
 * @note - Deletes receive and transmit queues
 * 
 * @return `p1p2_err_t` error code, always returns `P1P2_OK`
 * 
 */
p1p2_err_t p1p2_serial_deinit(void);

/**
 * Read a new message from the message queue
 * 
 * @param[out] message Pointer to `P1P2_Message_t`
 * @param[in] timeout Amount of ticks to wait for a new message to arrive
 * 
 * @return `p1p2_err_t` error code:
 *  - `P1P2_OK`: New message has been received
 *  - `P1P2_ERR_INVALID_STATE`: Receive message queue has not been initialized
 *  - `P1P2_ERR_INVALID_ARG`: Message cannot be a null pointer
 *  - `P1P2_ERR_TIMEOUT`: No new message has been received
 * 
 */
p1p2_err_t p1p2_message_read(P1P2_Message_t *message, uint32_t timeout);

/**
 * Write a new message to the message queue
 * 
 * @param[in] message Pointer to `P1P2_Message_t`
 * @param[in] timeout Amount of ticks to wait if queue is full
 * 
 * @return `p1p2_err_t` error code:
 *  - `P1P2_OK`: The message has been placed on the queue
 *  - `P1P2_ERR_INVALID_STATE`: Transmit message queue has not been initialized
 *  - `P1P2_ERR_INVALID_ARG`: Message cannot be a null pointer
 *  - `P1P2_ERR_TIMEOUT`: Message could not be placed on the queue
 * 
 */
p1p2_err_t p1p2_message_write(P1P2_Message_t *message, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* P1P2_SERIAL_H_ */