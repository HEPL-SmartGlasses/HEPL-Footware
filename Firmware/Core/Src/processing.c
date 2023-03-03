/*
 * processing.c
 *
 *  Created on: Feb 27, 2023
 *      Author: evanm
 */

#include "processing.h"

float w_avg_mag;

/*
 *  Matrix data
 */
float m_b0_f32[3] = {
		0.01, //TODO change to reflect actual distance between sensors
		0.01,
		0,
}; // Relative position of IMUs in board frame (b0, so relative to IMU0), unit (m)

float g_n_f32[3] = {
		0,
		0,
		g,
}; // Gravitation vector in frame n, unit (m/s^2)

float w_avg_f32[3] = {
		0,
		0,
		0,
}; // Average angular rate vector of b0 relative to nav in b0, unit (deg/s)

float q_f32[4] = {
		1,
		0,
		0,
		0,
}; // Quaternion representing rotation of board frame from nav frame, init to Identity quaternion (1, 0, 0, 0)

float rotation_b0_n_f32[9] = {
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
}; // Rotation matrix from board frame to nav frame, init to all zeros

float x_prev_f32[12] = {
		0,
		0,
		0,

		0,
		0,
		0,

		0,
		0,
		0,

		0,
		0,
		0,
}; // State variable at k-1, x = [r0_n, r1_n, v0_n, v1_n]^T, init to all zeros

float x_curr_f32[12] = {
		0,
		0,
		0,

		0,
		0,
		0,

		0,
		0,
		0,

		0,
		0,
		0,
}; // State variable at k, x = [r0_n, r1n, v0_n, v1_n]^T, init to all zeros

float F_matrix_f32[144] = {
		1,0,0,	0,0,0,	1,0,0,	0,0,0,
		0,1,0,	0,0,0,	0,1,0,	0,0,0,
		0,0,1,	0,0,0,	0,0,1,	0,0,0,

		0,0,0,	1,0,0,	0,0,0,	1,0,0,
		0,0,0,	0,1,0,	0,0,0,	0,1,0,
		0,0,0,	0,0,1,	0,0,0,	0,0,1,

		0,0,0,	0,0,0,	1,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,1,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,1,	0,0,0,

		0,0,0,	0,0,0,	0,0,0,	1,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,1,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,1,
}; // State transition matrix, both top right I matrices multiplied by dt

float F_transpose_f32[144]; // State transition matrix transpose

float B_matrix_f32[72] = {
		1,0,0, 0,0,0,
		0,1,0, 0,0,0,
		0,0,1, 0,0,0,

		0,0,0, 1,0,0,
		0,0,0, 0,1,0,
		0,0,0, 0,0,1,

		1,0,0, 0,0,0,
		0,1,0, 0,0,0,
		0,0,1, 0,0,0,

		0,0,0, 1,0,0,
		0,0,0, 0,1,0,
		0,0,0, 0,0,1,
}; // Control matrix to transform IMU data, top 2 I matrices multiplied by dt^2/2, bottom 2 multiplied by dt

float u_curr_f32[6] = {
		0,
		0,
		0,

		0,
		0,
		0,
}; // Input vector populated with IMU data in nav frame, init to all zeros, units (m/s^2)

/*
 *  Matrix instances
 */
arm_matrix_instance_f32 m_b0; 			// Relative IMU positions, frame b0
arm_matrix_instance_f32 g_n;			// Gravitation vector, frame n
arm_matrix_instance_f32 w_avg;			// Average angular rate vector, frame b0
arm_matrix_instance_f32 rotation_b0_n; 	// Rotation matrix, frame b0 to frame n
arm_matrix_instance_f32 x_prev;			// State variable, k-1
arm_matrix_instance_f32 x_curr;			// State variable, k
arm_matrix_instance_f32 F_matrix;		// State transition matrix
arm_matrix_instance_f32 F_transpose;	// State transition matrix transpose
arm_matrix_instance_f32 B_matrix;		// Control matrix
arm_matrix_instance_f32 u_curr;			// Input vector, frame n

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
	arm_mat_init_f32(&w_avg, numRows, numCols, w_avg_f32);

	// Rotation matrix
	numRows = 3;
	numCols = 3;
	arm_mat_init_f32(&rotation_b0_n, numRows, numCols, rotation_b0_n_f32);

	// k-1 State variable
	numRows = 12;
	numCols = 1;
	arm_mat_init_f32(&x_prev, numRows, numCols, x_prev_f32);

	// k State variable
	numRows = 12;
	numCols = 1;
	arm_mat_init_f32(&x_curr, numRows, numCols, x_curr_f32);

	// State transition matrix
	numRows = 12;
	numCols = 12;
	arm_mat_init_f32(&F_matrix, numRows, numCols, F_matrix_f32);

	// State transition matrix
	numRows = 12;
	numCols = 12;
	arm_mat_init_f32(&F_transpose, numRows, numCols, F_transpose_f32);
	arm_mat_trans_f32(&F_matrix, &F_transpose); // Fill transposed matrix

	// Control matrix
	numRows = 12;
	numCols = 6;
	arm_mat_init_f32(&B_matrix, numRows, numCols, B_matrix_f32);

	// Input vector
	numRows = 6;
	numCols = 1;
	arm_mat_init_f32(&u_curr, numRows, numCols, u_curr_f32);

}

void calculateCorrectedState(SensorData* IMU0_data, SensorData* IMU1_data, float timeDelta) {

	calculateAvgAngularRate(IMU0_data, IMU1_data); // w_avg

	calculateRotationMatrix(timeDelta); // R_b0_n

	updateFMatrix(timeDelta); // Update F, F^T with new timeDelta

	updateBMatrix(timeDelta); // Update B with new timeDelta

	updateUVector(IMU0_data, IMU1_data); // Update u_curr with IMU data

	calculateStateEstimation(); // x = F*x(k-1) + B*u(k) // TODO Make sure these multiplications happen as expected

	// calculate state estimation error covariance (P-)

	// Determine Swing or Stance Phase
		// calculate Observation vector (Zi), Observation matrix (Hi), Gain (Ki)

	// calculate optimal state estimation (x_best)

	// calculate optimal estimation error covariance (P)

	// update x_prev, P_prev, Q_prev

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

	float q1_3_scaling_term = arm_sin_f32(rotation_angle_div_2) / w_avg_mag; // reduce number of calculations

	// Determine change in rotation as quaternion
	float delta_q_f32[4];
	delta_q_f32[0] = arm_cos_f32(rotation_angle_div_2);
	delta_q_f32[1] = w_avg_f32[0] * q1_3_scaling_term;
	delta_q_f32[2] = w_avg_f32[1] * q1_3_scaling_term;
	delta_q_f32[3] = w_avg_f32[2] * q1_3_scaling_term;

	// Calculate new normalized quaternion
	arm_quaternion_product_single_f32(q_f32, delta_q_f32, q_f32); // q = q x delta_q
	arm_quaternion_normalize_f32(q_f32, q_f32, 1);	// q = q / |q|

	// Calculate rotation matrix from board frame to nav frame using quaternion
	arm_quaternion2rotation_f32(q_f32, rotation_b0_n_f32, 1);
}

void calculateStateEstimation() {
	float temp1_f32[12];
	arm_matrix_instance_f32 temp1;
	arm_mat_init_f32(&temp1, 12, 1, temp1_f32); // temp for first term of sum

	float temp2_f32[12];
	arm_matrix_instance_f32 temp2;
	arm_mat_init_f32(&temp2, 12, 1, temp2_f32); // temp for second term of sum

	arm_mat_mult_f32(&F_matrix, &x_prev, &temp1); // F*x(k-1) --> (12x12) * (12x1)

	arm_mat_mult_f32(&B_matrix, &u_curr, &temp2); // B*u(k) --> (12x6) * (6x1)

	arm_mat_sum_f32(&temp1, &temp2, &x_curr);
}

void updateFMatrix(float timeDelta) {
	int i;
	for(i = 0; i < 6; ++i) { // Update specific indices of F matrix
		F_matrix_f32[6 + (13*i)] = timeDelta;
	}

	arm_mat_trans_f32(&F_matrix, &F_transpose); // Update transpose matrix
}

void updateBMatrix(float timeDelta) {
	float dt2 = timeDelta * timeDelta / 2;

	int i;
	for(i = 0; i < 6; ++i) { // Update specific indices of B matrix with (timeDelta^2)/2
		B_matrix_f32[(7*i)] = dt2;
	}

	for(i = 0; i < 6; ++i) { // Update specific indices of B matrix with timeDelta
		B_matrix_f32[36 + (7*i)] = timeDelta;
	}
}

void updateUVector(SensorData* IMU0_data,SensorData* IMU1_data) {
	float temp0_f32[3] = {IMU0_data->XL_X, IMU0_data->XL_Y, IMU0_data->XL_Z}; // Init with IMU0 acceleration
	arm_matrix_instance_f32 temp0;
	arm_mat_init_f32(&temp0, 3, 1, temp0_f32); // temp for IMU0 vector

	float temp1_f32[3] = {IMU1_data->XL_X, IMU1_data->XL_Y, IMU1_data->XL_Z}; // Init with IMU1 acceleration
	arm_matrix_instance_f32 temp1;
	arm_mat_init_f32(&temp1, 3, 1, temp1_f32); // temp for IMU1 vector

	// Rotate IMU0 XL from board frame to nav frame
	arm_mat_mult_f32(&rotation_b0_n, &temp0, &temp0);

	// Rotate IMU1 XL from board frame to nav frame
	arm_mat_mult_f32(&rotation_b0_n, &temp1, &temp1);

	// Subtract gravitation vector from IMU0 XL vector
	arm_mat_sub_f32(&temp0, &g_n, &temp0);

	// Subtract gravitation vector from IMU1 XL vector
	arm_mat_sub_f32(&temp1, &g_n, &temp1);

	// Fill u input vector with IMU0 data
	u_curr_f32[0] = temp0_f32[0];
	u_curr_f32[1] = temp0_f32[1];
	u_curr_f32[2] = temp0_f32[2];

	// Fill u input vector with IMU1 data
	u_curr_f32[3] = temp1_f32[0];
	u_curr_f32[4] = temp1_f32[1];
	u_curr_f32[5] = temp1_f32[2];
}




