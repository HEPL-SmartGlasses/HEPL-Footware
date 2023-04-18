//
// xbee.c - SPI driver for XBee over STM32
// Maia Herrington (maiahe)

#include "xbee.h"

// define which XBee you are
#define GLASSBEE

// define addresses
#define GLASSBEE_ADDR 0x0013A2004176EAC6
#define FOOTBEE_ADDR 0x0013A2004185C377
#define COMPBEE_ADDR 0x0013A2004176E7FD

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
	if (data_size > 14) { data_size = 14; }

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


// XBeeTX(): transmits a given payload
//    Returns true if successful spi interaction
//    Modifies nothing
uint8_t XBeeTX
(
    uint8_t data[],
    uint8_t data_size, // in bytes
	uint8_t rx_buf[],
	uint8_t comp
)
{
	uint8_t frame[64];
	uint8_t frame_size = makeXBeeTXFrame(XBEE_TRANSMIT_FRAME, 0x01, data, data_size, frame, comp);
	HAL_GPIO_TogglePin(XBEE_CS_PORT, XBEE_CS_PIN);
	HAL_StatusTypeDef stat = HAL_SPI_TransmitReceive(XBEE_SPI_HANDLER, frame, rx_buf, frame_size, 2);
	HAL_GPIO_TogglePin(XBEE_CS_PORT, XBEE_CS_PIN);

	if (stat == HAL_OK) return 1;
	return 0;
}


// Receive interrupt
//void HAL_GPIO_EXTI_Callback(uint16_t pin)
//{
//	rx_size = 0;
//	while (!HAL_GPIO_ReadPin(XBEE_ATTN_PORT, XBEE_ATTN_PIN))
//	{
//	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
//		HAL_SPI_Receive(&hspi3, &xbee_rx_buf[rx_size], 1, 2);
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
//		rx_size = (rx_size + 1) % 32;
//	}
//}


// receives a given XBee command
// must be called when interrupt on SPI3_ATTN is low
// returns size of received packet
// NOTE:
// X = 4 bytes
// Y = 4 bytes
// State = 4 bytes
uint8_t parseXBeeRXFrame
(
	uint8_t xbee_rx_buf[],
	uint8_t rx_frame[]
)
{
//  if (rx_size > 0)
//  {
//	  int i = 0;
//	  int packet_size = 0;
//	  for(i = 0; i < rx_size; i++)
//	  {
//		  if (xbee_rx_buf[i] == 0x7e)
//		  {
//			  packet_size = (xbee_rx_buf[i+1] << 8) + xbee_rx_buf[i+2];
//		  }
//	  }
//  }
	return 0;
}


