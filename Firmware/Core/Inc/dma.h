/*
 * dma.h
 *
 *  Created on: Feb 23, 2023
 *      Author: evanm
 */

#ifndef INC_DMA_H_
#define INC_DMA_H_

#include "stm32f7xx_hal_dma.h"

#define DMA_BUFFER_SIZE 4096

typedef struct {
	uint8_t* rx_buffer;

} DMA_Info;

uint8_t dma_rx_xl1_buffer[DMA_BUFFER_SIZE] __attribute__((section(".dma_rx_xl1_buffer"))); // 4K DMA RX Buffer, Accelerometer 1
uint8_t dma_rx_xl2_buffer[DMA_BUFFER_SIZE] __attribute__((section(".dma_rx_xl2_buffer"))); // 4K DMA RX Buffer, Accelerometer 2

// DMA IMU read procedure:
// 1) Read FIFO Status Regs to get number of samples to transfer
// 2) Start DMA read transaction from FIFO buffer to DMA RX buffer, number of bytes determined from step 1
// 3) Interrupt when data is ready to process
// 4) Separate read data into sensor (XL, Gyro, Mag)

void DMA_StartTransfer(void);


#endif /* INC_DMA_H_ */
