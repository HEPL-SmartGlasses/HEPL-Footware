/*
 * IMU.c
 *
 *  Created on: Feb 18, 2023
 *      Author: evanm
 */

#include "IMU.h"

void IMU_init(SPI_HandleTypeDef* hspi, IMU IMU) {
	IMU.hspi = hspi;
	IMU.X_offset = 0;
	IMU.Y_offset = 0;
	IMU.Z_offset = 0;

	uint8_t buf[BUFFER_SIZE]; // Generic buffer for tx/rx

	// Verify SPI connection to IMU
	IMU_readRegister(IMU, WHO_AM_I, buf, 1);
	assert(buf[0] == WHO_I_AM_ID); // Crash if not connected properly

	// Enable Embedded Functions Bank A (allows sensor hub functions) - 10.1
	// ***All modifications of the embedded functions registers must happen in Power Down Mode
	buf[0] = FUNC_CFG_ACCESS;
	buf[1] = 0x80; // Enable Bank A
	IMU_writeRegister(IMU, buf, 1);

	// Config FIFO















	// TODO Do this last vvv & delete this comment
	// Config User Corrections
	IMU_updateOffsetCorrections(IMU);

}


float IMU_convertToMilliGs(int16_t reading) {
	// (Full-scale val / Max LSB val) * reading (LSB) = Accel (mg)
	return ((float)LA_FS / 0x8000) * reading;
}

void IMU_updateOffsetCorrections(IMU IMU) {
	// Read current accelerometer values
	uint8_t buffer[BUFFER_SIZE];
	IMU_readRegister(IMU, OUTX_L_XL, buffer, 6);

	int16_t x_value = (int16_t)((buffer[1] << 8) | buffer[0]);
	int16_t y_value = (int16_t)((buffer[3] << 8) | buffer[2]);
	int16_t z_value = (int16_t)((buffer[5] << 8) | buffer[4]);


}

HAL_StatusTypeDef IMU_readRegister(IMU IMU, uint8_t reg_addr, uint8_t* rx_buf, int num_bytes) {
	uint8_t reg_buffer[1] = {reg_addr};
	uint8_t rx[BUFFER_SIZE];

	__disable_irq();

	IMU_chipSelect();

	HAL_SPI_Transmit(IMU.hspi, (uint8_t *)reg_buffer, 1, SPI_TIMEOUT);
	HAL_SPI_Receive(IMU.hspi, (uint8_t *)rx, num_bytes, SPI_TIMEOUT);

	IMU_chipRelease();

	__enable_irq();
}

HAL_StatusTypeDef IMU_writeRegister(IMU IMU, uint8_t* tx_buf, int num_bytes) {
	__disable_irq();

	IMU_chipSelect();

	HAL_SPI_Transmit(IMU.hspi, (uint8_t *)tx_buf, num_bytes + 1, SPI_TIMEOUT);

	IMU_chipRelease();

	__enable_irq();
}

void IMU_chipSelect(void) {
	assert(0);
}

void IMU_chipRelease(void) {
	assert(0);
}


