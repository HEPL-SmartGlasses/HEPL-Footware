/*
 * IMU.c
 *
 *  Created on: Feb 18, 2023
 *      Author: evanm
 */

#include "IMU.h"

void IMU_init(SPI_HandleTypeDef* hspi, IMU* IMU) {
	IMU->hspi = hspi;
	IMU->X_offset = 0;
	IMU->Y_offset = 0;
	IMU->Z_offset = 0;

	uint8_t buf[BUFFER_SIZE]; // Generic buffer for tx/rx

	// Verify SPI connection to IMU
	IMU_readRegister(IMU, WHO_AM_I, buf, 1);
	assert(buf[0] == WHO_I_AM_ID); // Crash if not connected properly

	/*
	 *  Startup Sequence
	 */
	buf[0] = CTRL1_XL;
	buf[1] = 0x5C;
	IMU_writeRegister(IMU, buf, 1);

	buf[0] = CTRL2_G;
	buf[1] = 0x58;
	IMU_writeRegister(IMU, buf, 1);

	buf[0] = INT2_CTRL;
	buf[1] = 0x03;
	IMU_writeRegister(IMU, buf, 1);

	buf[0] = CTRL5_C;
	buf[1] = 0x60;
	IMU_writeRegister(IMU, buf, 1);

}

float IMU_convertAccel(uint8_t H_byte, uint8_t L_byte) {
	int16_t reading = (int16_t)(H_byte << 8) + L_byte;

	// (Full-scale val / Max LSB val) * reading (LSB) = Accel (g)
	return XL_SCALE_FACTOR * reading;
}

float IMU_convertGyro(uint8_t H_byte, uint8_t L_byte) {
	int16_t reading = (int16_t)(H_byte << 8) + L_byte;

	// (Full-scale val / Max LSB val) * reading (LSB) = Accel (g)
	return GYRO_SCALE_FACTOR * reading;
}

void IMU_readSensorData(IMU* IMU, SensorData* data) {
	uint8_t buf[BUFFER_SIZE];

	IMU_readRegister(IMU, OUTX_L_G, buf, 12);

	data->G_X = IMU_convertGyro(buf[1], buf[0]);
	data->G_Y = IMU_convertGyro(buf[3], buf[2]);
	data->G_Z = IMU_convertGyro(buf[5], buf[4]);

	data->XL_X = IMU_convertAccel(buf[7], buf[6]);
	data->XL_Y = IMU_convertAccel(buf[9], buf[8]);
	data->XL_Z = IMU_convertAccel(buf[11], buf[10]);
}

HAL_StatusTypeDef IMU_readRegister(IMU* IMU, uint8_t reg_addr, uint8_t* rx_buf, int num_bytes) {
	uint8_t reg_buffer[1] = {reg_addr};
	uint8_t rx[BUFFER_SIZE];

	HAL_StatusTypeDef status;

	__disable_irq();

	IMU_chipSelect();

	HAL_SPI_Transmit(IMU->hspi, (uint8_t *)reg_buffer, 1, SPI_TIMEOUT);
	status = HAL_SPI_Receive(IMU->hspi, (uint8_t *)rx, num_bytes, SPI_TIMEOUT);

	IMU_chipRelease();

	__enable_irq();

	return status;
}

HAL_StatusTypeDef IMU_writeRegister(IMU* IMU, uint8_t* tx_buf, int num_bytes) {
	HAL_StatusTypeDef status;

	__disable_irq();

	IMU_chipSelect();

	status = HAL_SPI_Transmit(IMU->hspi, (uint8_t *)tx_buf, num_bytes + 1, SPI_TIMEOUT);

	IMU_chipRelease();

	__enable_irq();

	return status;
}

void IMU_chipSelect(void) {
	assert(0);
}

void IMU_chipRelease(void) {
	assert(0);
}

void IMU_updateOffsetCorrections(IMU* IMU) {
	// Read current accelerometer values
	uint8_t buffer[BUFFER_SIZE];
	IMU_readRegister(IMU, OUTX_L_XL, buffer, 6);

	IMU->X_offset = (int16_t)((buffer[1] << 8) | buffer[0]);
	IMU->Y_offset = (int16_t)((buffer[3] << 8) | buffer[2]);
	IMU->Z_offset = (int16_t)((buffer[5] << 8) | buffer[4]);
}
