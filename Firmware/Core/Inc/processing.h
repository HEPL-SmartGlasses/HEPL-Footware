/*
 * processing.h
 *
 *  Created on: Feb 27, 2023
 *      Author: evanm
 */

#ifndef INC_PROCESSING_H_
#define INC_PROCESSING_H_

#include "arm_math.h"
#include "IMU.h"

#define g (float)9.80274 // Local acceleration due to gravity
#define deg2rad (float) 0.0174532925199 // pi / 180

/*
 *  Define matrix variables
 */
void init_processing(void);

/*
 *  State estimation procedure
 */
void calculateCorrectedState(SensorData* IMU0_data, SensorData* IMU1_data, float timeDelta);

/*
 *  Determine Avg Angular Rate from IMU data
 */
void calculateAvgAngularRate(SensorData* IMU0_data, SensorData* IMU1_data);

/*
 *  Determine Rotation matrix from board frame to nav frame
 */
void calculateRotationMatrix(float timeDelta);

/*
 *  Calculate state estimation (x) using the state equation
 *  	x(k) = F*x(k-1) + B*u(k)
 *  		F is state transition matrix
 *  		x(k-1) is prev state
 *  		B is control matrix to transform IMU data
 *  		u(k) is IMU data (Rotation_board_to_nav * accel_board(k) - g_nav) of both sensors
 */
void calculateStateEstimation();

/*
 *  Multiplies applicable Identity matrices by timeDelta, updates transpose too
 */
void updateFMatrix(float timeDelta);

/*
 *  Multiplies applicable Identity matrices by timeDelta^2/2 or timeDelta
 */
void updateBMatrix(float timeDelta);

/*
 *  Updates input vector using IMU data
 */
void updateUVector(SensorData* IMU0_data, SensorData* IMU1_data);

#endif /* INC_PROCESSING_H_ */
