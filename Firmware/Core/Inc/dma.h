/*
 * dma.h
 *
 *  Created on: Feb 23, 2023
 *      Author: evanm
 */

#ifndef INC_DMA_H_
#define INC_DMA_H_

#define DMA_BUFFER_SIZE 4096

uint8_t dma_rx_xl1_buffer[DMA_BUFFER_SIZE] __attribute__((section(".dma_rx_xl1_buffer"))); // 4K DMA RX Buffer, Accelerometer 1
uint8_t dma_rx_xl2_buffer[DMA_BUFFER_SIZE] __attribute__((section(".dma_rx_xl2_buffer"))); // 4K DMA RX Buffer, Accelerometer 2



#endif /* INC_DMA_H_ */
