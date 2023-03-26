/*
 * processing.h
 *
 *  Created on: Feb 27, 2023
 *      Author: evanm
 */

#ifndef INC_PROCESSING_H_
#define INC_PROCESSING_H_

#include "arm_math.h"
typedef struct {float XL_X;float XL_Y;float XL_Z;float G_X;float G_Y;float G_Z;} SensorData;

typedef struct {
	float X;
	float Y;
	float Z;
} Position;

#define g (float)9.80274 // Local acceleration due to gravity: Ann Arbor = 9.80274 //TODO
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
 *  Returns r_avg based on r0 & r1, units of meters
 */
float returnCurrentPosition(Position* current_pos);

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
 *  Calculate state estimation using the state equation
 *  	x(k) = F*x(k-1) + B*u(k)
 *  		x(k) is the current state estimation
 *  		F is the state transition matrix
 *  		x(k-1) is the previous state estimation
 *  		B is the control matrix to transform IMU data
 *  		u(k) is the IMU data (Rotation_board_to_nav * accel_board(k) - g_nav) of both sensors
 */
void calculateStateEstimation(void);

/*
 *  Calculate state estimation error covariance matrix ("A Priori Covariance")
 *  	P-(k) = F*P(k-1)*F^T + Q(k-1)
 *  		P-(k) is the current state estimation error covariance matrix (a priori covariance)
 *  		F is the state transition matrix
 *  		Q(k-1) is the previous process noise covariance matrix
 */
void calculateStateEstimationErrorCovariance(void);

/*
 *  Calculate Kalman Gain
 *  	Ki(k) = P-(k)*Hi^T * (Hi*P-(k)*Hi^T + Ri(k))^-1
 *  		Ki(k) is the current Kalman Gain of the current phase for the correction factor
 *  		P-(k) is the current state estimation error covariance matrix (a priori covariance)
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
 *  		x(k) is estimated current state, x_best represents the optimal (corrected) estimation
 *  		Ki(k) is the current Kalman Gain of the current phase for the correction factor
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
 *  Calculate optimal estimation error covariance matrix ("A Posteriori covariance")
 *  	P(k) = (I - Ki(k)*Hi)*P-(k)
 *  		P(k) is the current optimal estimation error covariance matrix (a posteriori covariance)
 *  		I is the 12x12 Identity matrix
 *  		Ki(k) is the current Kalman Gain of the current phase for the correction factor
 *  		Hi is the observation matrix of the current phase
 *  		P-(k) is the current state estimation error covariance matrix (a priori covariance)
 */
void calculateOptimalEstimationErrorCovariance(
		arm_matrix_instance_f32* Ki, /*(12xN)*/
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
void updatePreviousMatrices(void);

/*
 *  Do cross product of two inputed vectors
 */
void cross_product(
		arm_matrix_instance_f32* a,
		arm_matrix_instance_f32* b,
		arm_matrix_instance_f32* c);

void init_tuning(void);

#endif /* INC_PROCESSING_H_ */
