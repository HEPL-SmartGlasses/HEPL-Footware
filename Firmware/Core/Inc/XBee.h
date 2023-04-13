/*
 * XBee.h
 *
 *  Created on: Apr 4, 2023
 *      Author: evanm
 */

#ifndef INC_XBEE_H_
#define INC_XBEE_H_

#include "stdint.h"
#include "stm32l4xx_hal.h"

#define FOOTBEE_ADDR 0x0013A2004185C377
#define GLASSBEE_ADDR 0x0013A200410822FF
#define XBEE_SRC_ADDR FOOTBEE_ADDR
#define XBEE_DEST_ADDR GLASSBEE_ADDR
#define XBEE_START 0x7E
#define XBEE_TRANSMIT_FRAME 0x10
#define XBEE_RECEIVE_FRAME 0x90

extern const GPIO_TypeDef* XBEE_CS_PORT;
extern const int XBEE_CS_PIN;
extern SPI_HandleTypeDef XBEE_SPI;

void XBeeTransmitReceive(uint8_t* data_buf, uint8_t* xbee_tx_buf, uint8_t tx_data_size);

uint8_t makeXBeeFrame
(
		uint8_t frame_type,
		uint8_t frame_id,
		uint8_t data_size, // in bytes
		uint8_t data[],
		uint8_t frame[]
);

// returns frame size
// modifies frame[]. pass payload to be sent in data[].
uint8_t XBeeChecksum(uint8_t frame[], uint8_t frame_size);

void parseXBeeFrame (uint8_t* frame[]);

#endif /* INC_XBEE_H_ */
