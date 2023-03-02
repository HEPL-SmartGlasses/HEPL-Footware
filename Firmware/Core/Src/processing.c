/*
 * processing.c
 *
 *  Created on: Feb 27, 2023
 *      Author: evanm
 */

#include "processing.h"

float w_avg_mag;

/*
 *  Constant matrix data
 */
float m_b0_f32[3] = {
		0.01, //TODO change to reflect actual distance between sensors
		0.01,
		0
}; // Relative position of IMUs in board frame (b0, so relative to IMU0), unit (m)

float g_n_f32[3] = {
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

float rotation_b0_n_f32[9] = {
		0, 0, 0,
		0, 0, 0,
		0, 0, 0
}; // Rotation matrix from board frame to nav frame, init to all zeros

float x_prev_f32[12] = {
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0
}; // State variable at k-1, x = [r0_n, r1_n, v0_n, v1_n], init to all zeros

float x_curr_f32[12] = {
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0
}; // State variable at k, x = [r0_n, r1n, v0_n, v1_n], init to all zeros

/*
 *  Matrix instances
 */
arm_matrix_instance_f32 m_b0; 			// Relative IMU positions, frame b0
arm_matrix_instance_f32 g_n;			// Gravitation vector, frame n
arm_matrix_instance_f32 w_ang;			// Average angular rate vector, frame b0
arm_matrix_instance_f32 rotation_b0_n; 	// Rotation matrix, frame b0 to frame n
arm_matrix_instance_f32 x_prev;			// State variable, k-1
arm_matrix_instance_f32 x_curr;			// State variable, k

void init_processing(void) {
	uint32_t numRows, numCols; // temp vars

	/*
	 * Init vectors/matrices
	 */

	// Relative IMU position vector
	numRows = 3;
	numCols = 1;
	arm_mat_init_f32(&m_b0, numRows, numCols, m_b0_f32);

	// Gravitation vector
	numRows = 3;
	numCols = 1;
	arm_mat_init_f32(&g_n, numRows, numCols, g_n_f32);

	// Average angular rate vector
	numRows = 3;
	numCols = 1;
	arm_mat_init_f32(&w_ang, numRows, numCols, w_ang_f32);

	// Rotation matrix
	numRows = 3;
	numCols = 3;
	arm_mat_init_f32(&rotation_b0_n, numRows, numCols, rotation_b0_n_f32);

	// k-1 State variable
	numRows = 3;
	numCols = 4;
	arm_mat_init_f32(&x_prev, numRows, numCols, x_prev_f32);

	// k State variable
	numRows = 3;
	numCols = 4;
	arm_mat_init_f32(&x_curr, numRows, numCols, x_curr_f32);


}

void calculateAvgAngularRate(SensorData* IMU0_data, SensorData* IMU1_data) {
	w_avg_f32[0] = (IMU0_data->G_X + IMU1_data->G_X) / 2;
	w_avg_f32[1] = (IMU0_data->G_Y + IMU1_data->G_Y) / 2;
	w_avg_f32[2] = (IMU0_data->G_Z + IMU1_data->G_Z) / 2;

	// Determine |w_avg|
	w_avg_mag = (w_avg_f32[0]*w_avg_f32[0]) + (w_avg_f32[1]*w_avg_f32[1]) + (w_avg_f32[2]*w_avg_f32[2]);
	arm_sqrt_f32(w_avg_mag, &w_avg_mag);
}

void calculateRotationMatrix(float timeDelta) {
	// Determine change in rotation angle / 2(radians)
	float rotation_angle_div_2 = w_avg_mag * timeDelta * deg2rad / 2;

	float q1_3_scaling_term = arm_sin_f32(rotation_angle_div_2) / w_avg_mag;

	// Determine change in rotation as quaternion
	float delta_q_f32[4];
	delta_q_f32[0] = arm_cos_f32(rotation_angle_div_2);
	delta_q_f32[1] = w_avg_f32[0] * q1_3_scaling_term;
	delta_q_f32[2] = w_avg_f32[1] * q1_3_scaling_term;
	delta_q_f32[3] = w_avg_f32[2] * q1_3_scaling_term;

	// Calculate new normalized quaternion
	arm_quaternion_product_single_f32(q_f32, delta_q_f32, q_f32); // q = q x delta_q
	arm_quaternion_normalize(q_f32, q_f32, 1);	// q = q / |q|

	// Calculate rotation matrix from board frame to nav frame using quaternion
	arm_quaternion2rotation_f32(q_f32, rotation_b0_n_f32, 1);
}

