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
void calculateCorrectedState(
		SensorData* IMU0_data,
		SensorData* IMU1_data,
		float timeDelta);

/*
 *  Determine Avg Angular Rate from IMU data
 */
void calculateAvgAngularRate(
		SensorData* IMU0_data,
		SensorData* IMU1_data);

/*
 *  Determine Rotation matrix from board frame to nav frame
 */
void calculateRotationMatrix(
		float timeDelta);

/*
 *  Calculate state estimation (x) using the state equation
 *  	x(k) = F*x(k-1) + B*u(k)
 *  		F is the state transition matrix
 *  		x(k-1) is the previous state
 *  		B is the control matrix to transform IMU data
 *  		u(k) is the IMU data (Rotation_board_to_nav * accel_board(k) - g_nav) of both sensors
 */
void calculateStateEstimation();

/*
 *  Calculate Kalman Gain
 *  	Ki(k) = P-(k)*Hi^T * (Hi*P-(k)*Hi^T + Ri(k))^-1
 *  		Ki(k) is the current gain of the Kalman Gain of the current phase
 *  		P-(k) is the current state estimation error covariance matrix
 *  		Hi is the observation matrix of the current phase
 *  		Ri(k) is the current observation noise covariance matrix of the current phase
 */
void calculateGainMatrix(
		arm_matrix_instance_f32* Ki, /*(12xN)*/
		arm_matrix_instance_f32* Hi, /*(Nx12)*/
		arm_matrix_instance_f32* Ri /*(NxN)*/);

/*
 *  Calculate optimal state estimation (x_best) using the correction equation
 *  	x(k) <-- x_best(k) = x(k) + Ki(k)*(Zi(k) - Hi*x(k))
 *  		x(k) is estimated current state
 *  		Ki(k) is the calculated current gain for the correction factor
 *  		Zi(k) is the observation vector of the current phase
 *  		Hi is the observation matrix of the current phase
 *
 *  		Zi(k) - Hi*x(k) indicates the correction due to the current constraints
 *  			Swing phase: Velocity constraint (v1-v0 = w x m, Coriolis theorem), Position constraint (r1-r0 = m)
 *  			Stance phase: Velocity constraint (v1-v0 = w x m, Coriolis theorem), Position constraint (r1-r0 = m), ZUPT (v0, v1 = 0)
 *
 *  		Subscript i indicates swing or stance phase
 */
void calculateOptimalStateEstimation(
		arm_matrix_instance_f32* Ki, /*(12xN)*/
		arm_matrix_instance_f32* Zi, /*(Nx1)*/
		arm_matrix_instance_f32* Hi /*(Nx12)*/);

/*
 *  Multiplies applicable Identity matrices by timeDelta
 */
void updateFMatrix(
		float timeDelta);

/*
 *  Multiplies applicable Identity matrices by timeDelta^2/2 or timeDelta
 */
void updateBMatrix(
		float timeDelta);

/*
 *  Updates input vector using IMU data
 */
void updateUVector(
		SensorData* IMU0_data,
		SensorData* IMU1_data);

/*
 *  Updates Observation Vector Zi using angular rate and position constraints
 */
void updateZiVector(
		arm_matrix_instance_f32* Zi);

/*
 *  Update x(k-1), P(k-1), (Q(k-1)?)
 */
void updatePrevMatrices(void);

/*
 *  Do cross product of two inputed vectors
 */
void cross_product(
		arm_matrix_instance_f32* a,
		arm_matrix_instance_f32* b,
		arm_matrix_instance_f32* c);

#endif /* INC_PROCESSING_H_ */
