/*
 * IMU.h
 *
 *  Created on: Feb 18, 2023
 *      Author: evanm
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "stm32l4xx_hal.h"
#include "assert.h"

// Register Address Map
#define FUNC_CFG_ACCESS 			0x01 	// R/W Embedded functions config register - Default 0x00
#define SENSOR_SYNC_TIME_FRAME		0x04 	// R/W Sensor sync config register - Default 0x00
#define SENSOR_SYNC_RES_RATIO		0x05 	// R/W Sensor sync config register - Default 0x00
#define FIFO_CTRL1					0x06 	// R/W FIFO config register - Default 0x00
#define FIFO_CTRL2					0x07 	// R/W FIFO config register - Default 0x00
#define FIFO_CTRL3					0x08 	// R/W FIFO config register - Default 0x00
#define FIFO_CTRL4					0x09 	// R/W FIFO config register - Default 0x00
#define FIFO_CTRL5					0x0A 	// R/W FIFO config register - Default 0x00
#define DRDY_PULSE_CFG				0x0B 	// R/W - Default 0x00
#define INT1_CTRL					0x0D 	// R/W INT 1 pin control - Default 0x00
#define INT2_CTRL					0x0E 	// R/W INT 2 pin control - Default 0x00
#define WHO_AM_I					0x0F 	// R Who I Am ID - Default 0x6A
#define CTRL1_XL					0x10 	// R/W Accel & Gyro control register - Default 0x00
#define CTRL2_G						0x11 	// R/W Accel & Gyro control register - Default 0x00
#define CTRL3_C						0x12 	// R/W Accel & Gyro control register - Default 0x00
#define CTRL4_C						0x13 	// R/W Accel & Gyro control register - Default 0x00
#define CTRL5_C						0x14 	// R/W Accel & Gyro control register - Default 0x00
#define CTRL6_C						0x15 	// R/W Accel & Gyro control register - Default 0x00
#define CTRL7_G						0x16 	// R/W Accel & Gyro control register - Default 0x00
#define CTRL8_XL					0x17 	// R/W Accel & Gyro control register - Default 0x00
#define CTRL9_XL					0x18 	// R/W Accel & Gyro control register - Default 0xE0
#define CTRL10_C					0x19 	// R/W Accel & Gyro control register - Default 0x00
#define MASTER_CONFIG				0x1A 	// R/W I2C master config - Default 0x00
#define WAKE_UP_SRC					0x1B 	// R Interrupt register
#define TAP_SRC						0x1C 	// R Interrupt register
#define D6D_SRC						0x1D 	// R Interrupt register
#define STATUS_REG					0x1E 	// R Status data register for user interface and OIS data
#define OUT_TEMP_L					0x20 	// R Temperature output data register
#define OUT_TEMP_H					0x21 	// R Temperature output data register
#define OUTX_L_G					0x22 	// R Gyro X output register
#define OUTX_H_G					0x23 	// R Gyro X output register
#define OUTY_L_G					0x24 	// R Gyro Y output register
#define OUTY_H_G					0x25 	// R Gyro Y output register
#define OUTZ_L_G					0x26 	// R Gyro Z output register
#define OUTZ_H_G					0x27 	// R Gyro Z output register
#define OUTX_L_XL					0x28 	// R Accel X output register
#define OUTX_H_XL					0x29 	// R Accel X output register
#define OUTY_L_XL					0x2A 	// R Accel Y output register
#define OUTY_H_XL					0x2B 	// R Accel Y output register
#define OUTZ_L_XL					0x2C 	// R Accel Z output register
#define OUTZ_H_XL					0x2D 	// R Accel Z output register
#define SENSORHUB1_REG				0x2E	// R Sensor Hub output register
#define SENSORHUB2_REG				0x2F	// R Sensor Hub output register
#define SENSORHUB3_REG				0x30	// R Sensor Hub output register
#define SENSORHUB4_REG				0x31	// R Sensor Hub output register
#define SENSORHUB5_REG				0x32	// R Sensor Hub output register
#define SENSORHUB6_REG				0x33	// R Sensor Hub output register
#define SENSORHUB7_REG				0x34	// R Sensor Hub output register
#define SENSORHUB8_REG				0x35	// R Sensor Hub output register
#define SENSORHUB9_REG				0x36	// R Sensor Hub output register
#define SENSORHUB10_REG				0x37	// R Sensor Hub output register
#define SENSORHUB11_REG				0x38	// R Sensor Hub output register
#define SENSORHUB12_REG				0x39	// R Sensor Hub output register
#define FIFO_STATUS1				0x3A	// R FIFO status register
#define FIFO_STATUS2				0x3B	// R FIFO status register
#define FIFO_STATUS3				0x3C	// R FIFO status register
#define FIFO_STATUS4				0x3D	// R FIFO status register
#define FIFO_DATA_OUT_L				0x3E	// R FIFO data output register
#define FIFO_DATA_OUT_H				0x3F	// R FIFO data output register
#define TIMESTAMP0_REG				0x40	// R Timestamp output register
#define TIMESTAMP1_REG				0x41	// R Timestamp output register
#define TIMESTAMP2_REG				0x42	// R/W Timestamp output register
#define STEP_TIMESTAMP_L			0x49	// R Step counter timestamp register
#define STEP_TIMESTAMP_H			0x4A	// R Step counter timestamp register
#define STEP_COUNTER_L				0x4B	// R Step counter output register
#define STEP_COUNTER_H				0x4C	// R Step counter output register
#define SENSORHUB13_REG				0x4D	// R Sensor hub output register
#define SENSORHUB14_REG				0x4E	// R Sensor hub output register
#define SENSORHUB15_REG				0x4F	// R Sensor hub output register
#define SENSORHUB16_REG				0x50	// R Sensor hub output register
#define SENSORHUB17_REG				0x51	// R Sensor hub output register
#define SENSORHUB18_REG				0x52	// R Sensor hub output register
#define FUNC_SRC1					0x53	// R Interrupt register
#define FUNC_SRC2					0x54	// R Interrupt register
#define WRIST_TILT_IA				0x55	// R Interrupt register
#define TAP_CFG						0x58	// R/W Interrupt register - Default 0x00
#define TAP_THS_6D					0x59	// R/W Interrupt register - Default 0x00
#define INT_DUR2					0x5A	// R/W Interrupt register - Default 0x00
#define WAKE_UP_THS					0x5B	// R/W Interrupt register - Default 0x00
#define WAKE_UP_DUR					0x5C	// R/W Interrupt register - Default 0x00
#define FREE_FALL					0x5D	// R/W Interrupt register - Default 0x00
#define MD1_CFG						0x5E	// R/W Interrupt register - Default 0x00
#define MD2_CFG						0x5F	// R/W Interrupt register - Default 0x00
#define MASTER_CMD_CODE				0x60	// R/W - Default 0x00
#define SENS_SYNC_SPI_ERROR_CODE	0x61	// R/W - Default 0x00
#define OUT_MAG_RAW_X_L				0x66	// R External Mag X raw data output register
#define OUT_MAG_RAW_X_H				0x67	// R External Mag X raw data output register
#define OUT_MAG_RAW_Y_L				0x68	// R External Mag Y raw data output register
#define OUT_MAG_RAW_Y_H				0x69	// R External Mag Y raw data output register
#define OUT_MAG_RAW_Z_L				0x6A	// R External Mag Z raw data output register
#define OUT_MAG_RAW_Z_H				0x6B	// R External Mag Z raw data output register
#define INT_OIS						0x6F	// R/W - Default 0x00
#define CTRL1_OIS					0x70	// R/W OIS control register - Default 0x00
#define CTRL2_OIS					0x71	// R/W OIS control register - Default 0x00
#define CTRL3_OIS					0x72	// R/W OIS control register - Default 0x00
#define X_OFS_USR					0x73	// R/W Accelerometer user X offset correction - Default 0x00
#define Y_OFS_USR					0x74	// R/W Accelerometer user Y offset correction - Default 0x00
#define Z_OFS_USR					0x75	// R/W Accelerometer user Z offset correction - Default 0x00
// END Register Address Map

// Constants
#define g (float)9.80274
#define WHO_I_AM_ID 				0x6A
#define FULLSCALE_ACCEL				8		// Linear Acceleration Full-Scale (+/- 8g)
#define XL_SCALE_FACTOR				(float)g*FULLSCALE_ACCEL/32768	// Full-Scale Acceleration / Max LSB value (32768)
#define FULLSCALE_GYRO				1000	// Angular Rate Full-Scale (1000 deg/s)
#define GYRO_SCALE_FACTOR			(float)FULLSCALE_GYRO/32768	// Full-Scale Angular Rate / Max LSB value (32768)
#define SPI_TIMEOUT					0x01
#define BUFFER_SIZE					12

typedef struct {
	SPI_HandleTypeDef* hspi; // SPI bus

	// Accelerometer Offsets
	float X_offset;
	float Y_offset;
	float Z_offset;

	uint8_t chipID; // Integer representing IMU0,1

} IMU;

typedef struct {
	// Accel Data in Gs
	float XL_X;
	float XL_Y;
	float XL_Z;

	// Gyro Data in Deg/sec
	float G_X;
	float G_Y;
	float G_Z;
} SensorData;

/*
 *  @brief Initialize IMU
 *
 *	@param hspi: SPI bus of IMU
 *
 *	https://www.st.com/content/ccc/resource/technical/document/datasheet/76/27/cf/88/c5/03/42/6b/DM00218116.pdf/files/DM00218116.pdf/jcr:content/translations/en.DM00218116.pdf
 *
 */
void IMU_init(SPI_HandleTypeDef* hspi, IMU* IMU, uint8_t chipID);

/*
 *  @brief Read Gryo and Accel data from IMU
 *
 *  @param IMU: IMU struct
 *  @param data: struct to store sensor data
 */
void IMU_readSensorData(IMU* IMU, SensorData* data);

/*
 *  @brief Converts acceleration reading to g
 *
 *  @param 16-bit 2's complement reading, split into 2 bytes
 *  @retval Acceleration in g
 */
float IMU_convertAccel(uint8_t H_byte, uint8_t L_byte);

/*
 *  @brief Converts angular velocity reading to deg/s
 *
 *  @param 16-bit 2's complement reading, split into 2 bytes
 *  @retval Angular Velocity in deg/s
 */
float IMU_convertGyro(uint8_t H_byte, uint8_t L_byte);

/*
 *  @brief Read IMU register(s) over SPI
 *
 *	@param reg_addr: Address to read from (or starting address if consecutive)
 *	@param rx: Return buffer for read data
 *	@param num_bytes: Number of bytes to read
 *
 */
HAL_StatusTypeDef IMU_readRegister(IMU* IMU, uint8_t reg_addr, uint8_t* rx_buf, int num_bytes);

/*
 *  @brief Write IMU register(s) over SPI
 *
 *	@param tx: Buffer for data to write, reg_addr goes at index 0
 *	@param num_bytes: Number of bytes to write (excluding reg_addr)
 *
 */
HAL_StatusTypeDef IMU_writeRegister(IMU* IMU, uint8_t* tx_buf, int num_bytes);

/*
 *  @brief Enable IMU CS pin
 */
void IMU_chipSelect(uint8_t chipID);

/*
 *  @brief Disable IMU CS pin
 */
void IMU_chipRelease(uint8_t chipID);

void IMU_zero(IMU* imu0, IMU* imu1, IMU* imu2);

#endif /* INC_IMU_H_ */
