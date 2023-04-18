/*
 * XBee.c
 *
 *  Created on: Apr 4, 2023
 *      Author: evanm
 */

#include "XBee.h"

// define which XBee you are
#define FOOTBEE

// define addresses
#define GLASSBEE_ADDR 0x0013A2004176EAC6
#define FOOTBEE_ADDR 0x0013A2004185C377
#define SPARE_FOOTBEE_ADDR 0x0013A200410822FF
#define COMPBEE_ADDR 0x0013A2004176E7FD
#define BROADCAST_ADDR 0x000000000000FFFF

// and let the macros do the rest!
#ifdef GLASSBEE
#define XBEE_CS_PORT GPIOA
#define XBEE_CS_PIN GPIO_PIN_15
#define XBEE_ATTN_PORT GPIOB
#define XBEE_ATTN_PIN GPIO_PIN_6
#define XBEE_SRC_ADDR GLASSBEE_ADDR
#define XBEE_DEST_ADDR FOOTBEE_ADDR
extern SPI_HandleTypeDef hspi3;
#define XBEE_SPI_HANDLER &hspi3
#endif

#ifdef FOOTBEE
#define XBEE_CS_PORT GPIOA
#define XBEE_CS_PIN GPIO_PIN_15
#define XBEE_ATTN_PORT GPIOB
#define XBEE_ATTN_PIN GPIO_PIN_6
#define XBEE_SRC_ADDR FOOTBEE_ADDR
#define XBEE_DEST_ADDR GLASSBEE_ADDR
extern SPI_HandleTypeDef hspi3;
#define XBEE_SPI_HANDLER &hspi3
#endif

// externs
extern uint8_t xbee_rx_buf[32];
extern uint16_t rx_size;

uint8_t XBeeTransmitReceive(uint8_t* data_buf, uint8_t* xbee_rx_buf, uint8_t tx_data_size, uint8_t comp) {
	uint8_t xbee_tx_buf[32];

	uint8_t tx_size = makeXBeeTXFrame(XBEE_TRANSMIT_FRAME, 0x01, data_buf, tx_data_size, xbee_tx_buf, comp);

	// Set CS
	HAL_GPIO_WritePin(XBEE_CS_PORT, XBEE_CS_PIN, 0);

	__disable_irq();

	HAL_StatusTypeDef stat = HAL_SPI_TransmitReceive(XBEE_SPI_HANDLER, xbee_tx_buf, xbee_rx_buf, tx_size, 7);

	__enable_irq();

	// Clear CS
	HAL_GPIO_WritePin(XBEE_CS_PORT, XBEE_CS_PIN, 1);

	if (stat == HAL_OK) return 1;
	return 0;
}

// Calculates a checksum for a given XBee frame
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
} // XBeeChecksum()


// makeXBeeTXFrame(): Generates an XBee TX frame with the given data payload
// 	 Returns frame size
// 	 Modifies frame[]
uint8_t makeXBeeTXFrame
(
		uint8_t frame_type,
		uint8_t frame_id,
		uint8_t data[],
		uint8_t data_size, // in bytes
		uint8_t frame[],
		uint8_t comp
)
{
	// only do 14 bytes of data to avoid exceeding 32-byte frame size
	if (data_size > 16) { data_size = 16; }

	uint16_t frame_size = 0x0E + data_size;
	uint32_t checksum = 0;

	frame[0] = XBEE_START;
	frame[1] = ((frame_size) >> 8) & 0x00FF; // length upper byte
	frame[2] = ((frame_size) >> 0) & 0x00FF; // length lower byte
	frame[3] = frame_type;
	frame[4] = frame_id;
	if (comp == 1)
	{
		for (int i = 0; i < 8; i++) // write 64-bit dest
		{
			uint8_t temp = (COMPBEE_ADDR >> 8*(7-i));
			frame[i + 5] = temp;
		}
	}
	else
	{
		for (int i = 0; i < 8; i++) // write 64-bit dest
		{
			uint8_t temp = (XBEE_DEST_ADDR >> 8*(7-i));
			frame[i + 5] = temp;
		}
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

/*void parseXbeeData(){

	int i = 0;
	while (xbee_rx_buf[i] != 0x7e) {
		i++;
	}

	int frame = xbee_rx_buf[i + 3];

	if (frame != 0x90){
		return;
	}

	int xInit = i + 15, yInit = i + 19, sInit = i + 23;
	//int xEnd = 3, yEnd = 7, sEnd = 11;
	int packetSize = 4;

	int x = 0;
	int y = 0;

	for (int j = 0; j < packetSize; j++){
		x |= xbee_rx_buf[xInit + j] << ((3-j)*8);
		y |= xbee_rx_buf[yInit + j] << ((3-j)*8);
		status |= xbee_rx_buf[sInit + j] << ((3-j)*8);

	}

}
*/
