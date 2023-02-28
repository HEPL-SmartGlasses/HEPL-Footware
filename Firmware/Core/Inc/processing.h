/*
 * processing.h
 *
 *  Created on: Feb 27, 2023
 *      Author: evanm
 */

#ifndef INC_PROCESSING_H_
#define INC_PROCESSING_H_

#include "arm_math.h"

#define g (float)9.80274 // Local acceleration due to gravity

float rotation_angle = 0;

/*
 *  Constant matrix data
 */
const float m_b0_f32[3] = {
		0.01, //TODO change to reflect actual distance between sensors
		0.01,
		0
}; // Relative position of IMUs in board frame (b0, so relative to IMU0), unit (m)

const float g_n_f32[3] = {
		0,
		0,
		g
}; // Gravitation vector in frame n, unit (m/s^2)

/*
 *  Mutable matrix data
 */
float w_avg_f32[3] = {
		0,
		0,
		0
}; // Average angular rate vector of b0 relative to nav in b0, unit (deg/s)

float q_f32[4] = {
		1,
		0,
		0,
		0
}; // Quaternion representing rotation of board frame from nav frame, init to Identity quaternion (1, 0, 0, 0)

float x_prev_f32[12] = {
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0
}; // State variable at k-1, x = [r0_n, r1_n, v0_n, v1_n], init to all zeros

float x_curr_f32[12] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
}; // State variable at k, x = [r0_n, r1n, v0_n, v1_n], init to all zeros

/*
 *  Matrix instances
 */
arm_matrix_instance_f32 m_b0; 	// Relative IMU positions, frame b0
arm_matrix_instance_f32 g_n;	// Gravitation vector, frame n
arm_matrix_instance_f32 w_ang;	// Average angular rate vector, frame b0
arm_matrix_instance_f32 q;		// Quaternion, frame n to frame b0
arm_matrix_instance_f32 x_prev;	// State variable, k-1
arm_matrix_instance_f32 x_curr;	// State variable, k



void init_processing(void);

/*
 *  Determine Avg Angular Rate from IMU data
 */
void calculateAvgAngularRate(SensorData* IMU0_data, SensorData* IMU1_data);

/*
 *  Determine Rotation matrix from board frame to nav frame
 */
void calculateRotationMatrix(float timeDelta);



#endif /* INC_PROCESSING_H_ */
