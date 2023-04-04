/*
 * IMU.c
 *
 *  Created on: Feb 18, 2023
 *      Author: evanm
 */

#include "IMU.h"

int8_t IMU_offsets[9] = {
		282,334,505,
		243,363,412,
		611,-20,52,
};

void IMU_init(SPI_HandleTypeDef* hspi, IMU* IMU, uint8_t chipID) {
	IMU->hspi = hspi;
	IMU->X_offset = IMU_offsets[3*chipID + 0];
	IMU->Y_offset = IMU_offsets[3*chipID + 1];
	IMU->Z_offset = IMU_offsets[3*chipID + 2];
	IMU->chipID = chipID;

	uint8_t buf[BUFFER_SIZE]; // Generic buffer for tx/rx

	// Set to 4-wire
	buf[0] = CTRL3_C;
	buf[1] = 0x04;
	IMU_writeRegister(IMU, buf, 1);

	// Verify SPI connection to IMU
	IMU_readRegister(IMU, WHO_AM_I, buf, 1);
	assert(buf[0] == WHO_I_AM_ID); // Crash if not connected properly

	/*
	 *  Startup Sequence
	 */
	buf[0] = CTRL1_XL;
	buf[1] = 0x4C;
	IMU_writeRegister(IMU, buf, 1);

	buf[0] = CTRL2_G;
	buf[1] = 0x48;
	IMU_writeRegister(IMU, buf, 1);

	buf[0] = INT2_CTRL;
	buf[1] = 0x03;
	IMU_writeRegister(IMU, buf, 1);

	buf[0] = CTRL5_C;
	buf[1] = 0x60;
	IMU_writeRegister(IMU, buf, 1);

	buf[0] = CTRL6_C;
	buf[1] = 0x00;
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
	uint8_t reg_buffer[1] = {reg_addr | 0x80};

	HAL_StatusTypeDef status;

	__disable_irq();

	IMU_chipSelect(IMU->chipID);

	HAL_SPI_Transmit(IMU->hspi, (uint8_t *)reg_buffer, 1, SPI_TIMEOUT);
	status = HAL_SPI_Receive(IMU->hspi, (uint8_t *)rx_buf, num_bytes, SPI_TIMEOUT);

	IMU_chipRelease(IMU->chipID);

	__enable_irq();

	return status;
}

HAL_StatusTypeDef IMU_writeRegister(IMU* IMU, uint8_t* tx_buf, int num_bytes) {
	HAL_StatusTypeDef status;

	__disable_irq();

	IMU_chipSelect(IMU->chipID);

	status = HAL_SPI_Transmit(IMU->hspi, (uint8_t *)tx_buf, num_bytes + 1, SPI_TIMEOUT);

	IMU_chipRelease(IMU->chipID);

	__enable_irq();

	return status;
}

void IMU_chipSelect(uint8_t chipID) {
	HAL_GPIO_WritePin(GPIOB, (1 << chipID), 0); // PB0,1,2 for IMU 0,1,2
}

void IMU_chipRelease(uint8_t chipID) {
	HAL_GPIO_WritePin(GPIOB, (1 << chipID), 1); // PB0,1,2 for IMU 0,1,2
}

void IMU_zero(IMU* imu0, IMU* imu1, IMU* imu2) {
	uint8_t buf[12];

	buf[0] = X_OFS_USR;
	buf[1] = imu0->X_offset;
	buf[2] = imu0->Y_offset;
	buf[3] = imu0->Z_offset;
	IMU_writeRegister(imu0, buf, 3);

	buf[0] = X_OFS_USR;
	buf[1] = imu1->X_offset;
	buf[2] = imu1->Y_offset;
	buf[3] = imu1->Z_offset;
	IMU_writeRegister(imu1, buf, 3);

	buf[0] = X_OFS_USR;
	buf[1] = imu2->X_offset;
	buf[2] = imu2->Y_offset;
	buf[3] = imu2->Z_offset;
	IMU_writeRegister(imu2, buf, 3);

}
