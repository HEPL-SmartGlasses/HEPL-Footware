//
// xbee.c - SPI driver for XBee over STM32
// Maia Herrington (maiahe)

#include "stm32l4xx_hal.h"

#define XBEE_START 0x7E
#define XBEE_TRANSMIT_FRAME 0x10
#define XBEE_RECEIVE_FRAME 0x90

uint8_t XBeeChecksum(uint8_t frame[], uint8_t frame_size);

uint8_t makeXBeeTXFrame
(
		uint8_t frame_type,
		uint8_t frame_id,
		uint8_t data[],
		uint8_t data_size, // in bytes
		uint8_t frame[],
		uint8_t comp
);

uint8_t XBeeTX
(
    uint8_t data[],
    uint8_t data_size, // in bytes
	uint8_t rx_buf[],
	uint8_t comp
);


uint8_t parseXBeeRXFrame
(
	uint8_t xbee_rx_buf[],
	uint8_t rx_frame[]
);

