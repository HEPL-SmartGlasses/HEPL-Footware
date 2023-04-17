/*
 * XBee.c
 *
 *  Created on: Apr 4, 2023
 *      Author: evanm
 */

#include "XBee.h"

void XBeeTransmitReceive(uint8_t* data_buf, uint8_t* xbee_rx_buf, uint8_t tx_data_size, uint64_t dest_addr) {
	uint8_t xbee_tx_buf[32];

	uint8_t tx_size = makeXBeeFrame(XBEE_TRANSMIT_FRAME, 0x01, tx_data_size, data_buf, xbee_tx_buf, dest_addr);

	// Set CS
	HAL_GPIO_WritePin(XBEE_CS_PORT, XBEE_CS_PIN, 0);

	__disable_irq();

	HAL_SPI_TransmitReceive(&XBEE_SPI, xbee_tx_buf, xbee_rx_buf, tx_size, 7);

	__enable_irq();

	// Clear CS
	HAL_GPIO_WritePin(XBEE_CS_PORT, XBEE_CS_PIN, 1);

}

// returns frame size
// modifies frame[]. pass payload to be sent in data[].
uint8_t XBeeChecksum(uint8_t frame[], uint8_t frame_size)
{
	uint8_t checksum = 0;
	uint8_t ret = 0;
	for (int i = 0; i < frame_size; i++) // skip bytes 0-2, and last
	{
		uint8_t temp = frame[i+3];
		checksum += temp;
	}
	checksum = 0x00FF - checksum;
	ret = (uint8_t)(checksum & 0x00FF);
	return ret;
}

uint8_t makeXBeeFrame(
			uint8_t frame_type,
			uint8_t frame_id,
			uint8_t data_size, // in bytes
			uint8_t data[],
			uint8_t frame[],
			uint64_t dest_addr
){
	// only do 14 bytes of data to avoid exceeding x-byte frame size
	if (data_size > 20) { data_size = 20; }

	uint16_t frame_size = 0x0E + data_size;
	uint32_t checksum = 0;

	frame[0] = XBEE_START;
	frame[1] = ((frame_size) >> 8) & 0x00FF; // length upper byte
	frame[2] = ((frame_size) >> 0) & 0x00FF; // length lower byte
	frame[3] = frame_type;
	frame[4] = frame_id;
	for (int i = 0; i < 8; i++) // write 64-bit dest
	{
		uint8_t temp = (dest_addr >> 8*(7-i));
		frame[i + 5] = temp;
	}
	frame[13] = 0xFF; // 16-bit addr upper
	frame[14] = 0xFE; // 16-bit addr lower
	frame[15] = 0x00; // broadcast_radius
	frame[16] = 0x00; // options
	for (int i = 0; i < data_size; i++) // add data payload to frame
	{
		uint8_t temp = data[i];
		frame[i + 17] = temp;
	}

	// update checksum
	checksum = XBeeChecksum(frame, frame_size);
	frame[data_size + 17] = checksum;
	return frame_size + 4;
}

void parseXBeeFrame (uint8_t* frame[])
{
}
