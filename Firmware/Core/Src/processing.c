/*
 * processing.c
 *
 *  Created on: Feb 27, 2023
 *      Author: evanm
 */

#include <processing.h>
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "assert.h"

ZUPTNode* ZUPTHead; // Head to ZUPT linked list

float w_avg_b0_mag; // Magnitude of average angular rate, board frame

float w_avg_x_ring[RING_SIZE];
float w_avg_y_ring[RING_SIZE];
float w_avg_z_ring[RING_SIZE];

float xl0_avg_x_ring[RING_SIZE];
float xl0_avg_y_ring[RING_SIZE];
float xl0_avg_z_ring[RING_SIZE];
float xl1_avg_x_ring[RING_SIZE];
float xl1_avg_y_ring[RING_SIZE];
float xl1_avg_z_ring[RING_SIZE];

uint8_t w_oldest = 0;
uint8_t xl_oldest = 0;

/*
 *  Matrix data
 */
const float m_b0_f32[3] = { // (3x1)
		-0.0127,
		+0.0127,
		0,
}; // Relative position of IMUs in board frame (b0, so relative to IMU0), unit (m)

const float g_n_f32[3] = { // (3x1)
		0,
		0,
		g,
}; // Gravitation vector in frame n, unit (m/s^2)

float xl_b0_f32[3] = { // (3x1)
		0,
		0,
		0,
}; // Average acceleration vector of b0 relative to nav in b0

float xl_b1_f32[3] = { // (3x1)
		0,
		0,
		0,
}; // Average acceleration vector of b1 relative to nav in b0

float w_avg_b0_f32[3] = { // (3x1)
		0,
		0,
		0,
}; // Average angular rate vector of b0 relative to nav in b0, unit (deg/s)

float q_f32[4] = { // (4x1)
		1,
		0,
		0,
		0,
}; // Quaternion representing rotation of board frame from nav frame, init to Identity quaternion (1, 0, 0, 0)

float rotation_b0_n_f32[9] = { // (3x3)
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
}; // Rotation matrix from board frame to nav frame, init to all zeros

float x_prev_f32[12] = { // (12x1)
		0,
		0,
		0,

		-0.0127,
		+0.0127,
		0,

		0,
		0,
		0,

		0,
		0,
		0,
}; // State variable at k-1, x = [r0_n, r1_n, v0_n, v1_n]^T, init to all zeros

float x_curr_f32[12] = { // (12x1)
		0,
		0,
		0,

		-0.0127,
		+0.0127,
		0,

		0,
		0,
		0,

		0,
		0,
		0,
}; // State variable at k, x = [r0_n, r1_n, v0_n, v1_n]^T

const float x_init_f32[12] = { // (12x1)
		0,
		0,
		0,

		-0.0127,
		+0.0127,
		0,

		0,
		0,
		0,

		0,
		0,
		0,
}; // State variable at k, x = [r0_n, r1_n, v0_n, v1_n]^T, used for RESET

float F_matrix_f32[144] = { // (12x12)
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

float B_matrix_f32[72] = { // (12x6)
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

float u_curr_f32[6] = { // (6x1)
		0,
		0,
		0,

		0,
		0,
		0,
}; // Input vector populated with IMU data in nav frame, init to all zeros, units (m/s^2)

const float H_swing_f32[72] = { // (6x12)
		0,0,0, 0,0,0, -1,0,0, 1,0,0,
		0,0,0, 0,0,0, 0,-1,0, 0,1,0,
		0,0,0, 0,0,0, 0,0,-1, 0,0,1,

		-1,0,0, 1,0,0, 0,0,0, 0,0,0,
		0,-1,0, 0,1,0, 0,0,0, 0,0,0,
		0,0,-1, 0,0,1, 0,0,0, 0,0,0,
}; // Observation matrix H_1, swing phase

const float H_stance_f32[144] = { // (12x12)
		0,0,0, 0,0,0, -1,0,0, 1,0,0,
		0,0,0, 0,0,0, 0,-1,0, 0,1,0,
		0,0,0, 0,0,0, 0,0,-1, 0,0,1,

		-1,0,0, 1,0,0, 0,0,0, 0,0,0,
		0,-1,0, 0,1,0, 0,0,0, 0,0,0,
		0,0,-1, 0,0,1, 0,0,0, 0,0,0,

		0,0,0, 0,0,0, 1,0,0, 0,0,0,
		0,0,0, 0,0,0, 0,1,0, 0,0,0,
		0,0,0, 0,0,0, 0,0,1, 0,0,0,

		0,0,0, 0,0,0, 0,0,0, 1,0,0,
		0,0,0, 0,0,0, 0,0,0, 0,1,0,
		0,0,0, 0,0,0, 0,0,0, 0,0,1,
}; // Observation matrix H_2, stance phase

float Z_swing_f32[6] = { // (6x1)
		0,
		0,
		0,

		0,
		0,
		0,
}; // Observation matrix Z, swing phase, init to all zeros

float Z_stance_f32[12] = { // (12x1)
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
}; // Observation vector Z, stance phase, init to all zeros (last 2 vectors stay as zero)

float K_swing_f32[72] = { // (12x6)
		0,0,0, 0,0,0,
		0,0,0, 0,0,0,
		0,0,0, 0,0,0,

		0,0,0, 0,0,0,
		0,0,0, 0,0,0,
		0,0,0, 0,0,0,

		0,0,0, 0,0,0,
		0,0,0, 0,0,0,
		0,0,0, 0,0,0,

		0,0,0, 0,0,0,
		0,0,0, 0,0,0,
		0,0,0, 0,0,0,
}; // Kalman Gain K_1, swing phase, init to all zeros

float K_stance_f32[144] = { // (12x12)
		0,0,0,	0,0,0,	0,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,0,

		0,0,0,	0,0,0,	0,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,0,

		0,0,0,	0,0,0,	0,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,0,

		0,0,0,	0,0,0,	0,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,0,
}; // Kalman Gain K_2, stance phase, init to all zeros

#define xVar 0.41569940440904957 * 2
#define yVar 0.3544485509713772 * 2
#define zVar 0.7406725369262939 * 2
#define xyCovar 0.01641281882542819 * 2
#define xzCovar -0.41979413683597244 * 2
#define yzCovar -0.34141210838259917 * 2

#define dT 1/50
#define dT2 dT*dT
#define dT3 dT2*dT

#define r_tune 5

uint8_t diverging_flag = 0;

const float R_swing_f32[36] = { // (6x6)
		xVar,xyCovar,xzCovar,	0,0,0,
		xyCovar,yVar,yzCovar,	0,0,0,
		xzCovar,yzCovar,zVar,	0,0,0,

		0,0,0,		xVar,xyCovar,xzCovar,
		0,0,0,		xyCovar,yVar,yzCovar,
		0,0,0,		xzCovar,yzCovar,zVar,
}; // Observation noise covariance, swing phase, init to // TODO fill this in

const float R_stance_f32[144] = { // (12x12)
		xVar,xyCovar,xzCovar,	0,0,0,	0,0,0,	0,0,0,
		xyCovar,yVar,yzCovar,	0,0,0,	0,0,0,	0,0,0,
		xzCovar,yzCovar,zVar,	0,0,0,	0,0,0,	0,0,0,

		0,0,0,	xVar,xyCovar,xzCovar,	0,0,0,	0,0,0,
		0,0,0,	xyCovar,yVar,yzCovar,	0,0,0,	0,0,0,
		0,0,0,	xzCovar,yzCovar,zVar,	0,0,0,	0,0,0,

		0,0,0,	0,0,0,	0,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,0,

		0,0,0,	0,0,0,	0,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,0,
}; // Observation noise covariance, stance phase, init to // TODO fill this in

float P_prev_f32[144] = { // (12x12)
		0.5*dT3,0,0,	0,0,0,	0,0,0,	0,0,0,
						0,0.5*dT3,0,	0,0,0,	0,0,0,	0,0,0,
						0,0,0.5*dT3,	0,0,0,	0,0,0,	0,0,0,

						0,0,0,	0.5*dT3,0,0,	0,0,0,	0,0,0,
						0,0,0,	0,0.5*dT3,0,	0,0,0,	0,0,0,
						0,0,0,	0,0,0.5*dT3,	0,0,0,	0,0,0,

						0,0,0,	0,0,0,	1.5*dT2/100,0,0,	0,0,0,
						0,0,0,	0,0,0,	0,1.5*dT2/100,0,	0,0,0,
						0,0,0,	0,0,0,	0,0,1.5*dT2/100,	0,0,0,

						0,0,0,	0,0,0,	0,0,0,	1.5*dT2/100,0,0,
						0,0,0,	0,0,0,	0,0,0,	0,1.5*dT2/100,0,
						0,0,0,	0,0,0,	0,0,0,	0,0,1.5*dT2/100,
}; // A posteriori covariance, k-1

float P_curr_f32[144] = { // (12x12)
		0.5*dT3,0,0,	0,0,0,	0,0,0,	0,0,0,
						0,0.5*dT3,0,	0,0,0,	0,0,0,	0,0,0,
						0,0,0.5*dT3,	0,0,0,	0,0,0,	0,0,0,

						0,0,0,	0.5*dT3,0,0,	0,0,0,	0,0,0,
						0,0,0,	0,0.5*dT3,0,	0,0,0,	0,0,0,
						0,0,0,	0,0,0.5*dT3,	0,0,0,	0,0,0,

						0,0,0,	0,0,0,	1.5*dT2/100,0,0,	0,0,0,
						0,0,0,	0,0,0,	0,1.5*dT2/100,0,	0,0,0,
						0,0,0,	0,0,0,	0,0,1.5*dT2/100,	0,0,0,

						0,0,0,	0,0,0,	0,0,0,	1.5*dT2/100,0,0,
						0,0,0,	0,0,0,	0,0,0,	0,1.5*dT2/100,0,
						0,0,0,	0,0,0,	0,0,0,	0,0,1.5*dT2/100,
}; // A posteriori covariance, k

float P_init_f32[144] = { // (12x12)
		0.5*dT3,0,0,	0,0,0,	0,0,0,	0,0,0,
				0,0.5*dT3,0,	0,0,0,	0,0,0,	0,0,0,
				0,0,0.5*dT3,	0,0,0,	0,0,0,	0,0,0,

				0,0,0,	0.5*dT3,0,0,	0,0,0,	0,0,0,
				0,0,0,	0,0.5*dT3,0,	0,0,0,	0,0,0,
				0,0,0,	0,0,0.5*dT3,	0,0,0,	0,0,0,

				0,0,0,	0,0,0,	1.5*dT2/100,0,0,	0,0,0,
				0,0,0,	0,0,0,	0,1.5*dT2/100,0,	0,0,0,
				0,0,0,	0,0,0,	0,0,1.5*dT2/100,	0,0,0,

				0,0,0,	0,0,0,	0,0,0,	1.5*dT2/100,0,0,
				0,0,0,	0,0,0,	0,0,0,	0,1.5*dT2/100,0,
				0,0,0,	0,0,0,	0,0,0,	0,0,1.5*dT2/100,
}; // A posteriori covariance, k

float P_minus_f32[144] = { // (12x12)
		0.5*dT3,0,0,	0,0,0,	0,0,0,	0,0,0,
						0,0.5*dT3,0,	0,0,0,	0,0,0,	0,0,0,
						0,0,0.5*dT3,	0,0,0,	0,0,0,	0,0,0,

						0,0,0,	0.5*dT3,0,0,	0,0,0,	0,0,0,
						0,0,0,	0,0.5*dT3,0,	0,0,0,	0,0,0,
						0,0,0,	0,0,0.5*dT3,	0,0,0,	0,0,0,

						0,0,0,	0,0,0,	1.5*dT2/100,0,0,	0,0,0,
						0,0,0,	0,0,0,	0,1.5*dT2/100,0,	0,0,0,
						0,0,0,	0,0,0,	0,0,1.5*dT2/100,	0,0,0,

						0,0,0,	0,0,0,	0,0,0,	1.5*dT2/100,0,0,
						0,0,0,	0,0,0,	0,0,0,	0,1.5*dT2/100,0,
						0,0,0,	0,0,0,	0,0,0,	0,0,1.5*dT2/100,
}; // A priori covariance, k

float Q_prev_f32[144] = { // (12x12)
		0.5*dT3,0,0,	0,0,0,	0.75*dT2,0,0,	0,0,0,
		0,0.5*dT3,0,	0,0,0,	0,0.75*dT2,0,	0,0,0,
		0,0,0.5*dT3,	0,0,0,	0,0,0.75*dT2,	0,0,0,

		0,0,0,	0.5*dT3,0,0,	0,0,0,	0.75*dT2,0,0,
		0,0,0,	0,0.5*dT3,0,	0,0,0,	0,0.75*dT2,0,
		0,0,0,	0,0,0.5*dT3,	0,0,0,	0,0,0.75*dT2,

		0.75*dT2,0,0,	0,0,0,	1.5*dT,0,0,	0,0,0,
		0,0.75*dT2,0,	0,0,0,	0,1.5*dT,0,	0,0,0,
		0,0,0.75*dT2,	0,0,0,	0,0,1.5*dT,	0,0,0,

		0,0,0,	0.75*dT2,0,0,	0,0,0,	1.5*dT,0,0,
		0,0,0,	0,0.75*dT2,0,	0,0,0,	0,1.5*dT,0,
		0,0,0,	0,0,0.75*dT2,	0,0,0,	0,0,1.5*dT,
}; // Process noise covariance, k-1, init to stuff

/*
 *  Temp arrays
 */

float temp12x1_0_f32[12];
float temp12x1_1_f32[12];

float tempNx12_0_f32[144];
float temp12xN_0_f32[144];
float temp12xN_1_f32[144];

float tempNxN_0_f32[144];
float tempNxN_1_f32[144];
float tempNxN_2_f32[144];

/*
 *  Debug arrays
 */
int phase_out = 0;
float optimal_f32[3] = {0,0,0,};	// x, y, z
float correction_f32[3] = {0,0,0,}; // x, y, z
float prediction_f32[3] = {0,0,0,}; // x, y, z
float gain_f32[3] = {0,0,0,}; // x, y, z

// ZUPT
enum PHASE curr_phase = STANCE;

// Saturating interval counter
uint8_t phase_counter = 0; // 0 --> stance side, PHASE_INTERVAL_THRESHOLD --> swing side



/*
 *  Matrix instances
 */
arm_matrix_instance_f32 m_b0; 			// Relative IMU positions, frame b0
arm_matrix_instance_f32 g_n;			// Gravitation vector, frame n
arm_matrix_instance_f32 w_avg_b0;		// Average angular rate vector, frame b0, k
arm_matrix_instance_f32 rotation_b0_n; 	// Rotation matrix, frame b0 to frame n
arm_matrix_instance_f32 x_prev;			// State variable, k-1
arm_matrix_instance_f32 x_curr;			// State variable, k
arm_matrix_instance_f32 F_matrix;		// State transition matrix
arm_matrix_instance_f32 B_matrix;		// Control matrix
arm_matrix_instance_f32 u_curr;			// Input vector, frame n, k
arm_matrix_instance_f32 H_swing;		// Observation matrix, swing phase
arm_matrix_instance_f32 H_stance;		// Observation matrix, stance phase
arm_matrix_instance_f32 Z_swing;		// Observation vector, swing phase, k
arm_matrix_instance_f32 Z_stance;		// Observation vector, stance phase, k
arm_matrix_instance_f32 K_swing;		// Kalman Gain, swing phase, k
arm_matrix_instance_f32 K_stance;		// Kalman Gain, stance phase, k
arm_matrix_instance_f32 R_swing;		// Observation noise covariance, swing phase, k
arm_matrix_instance_f32 R_stance;		// Observation noise covariance, stance phase, k
arm_matrix_instance_f32 P_prev;			// A posteriori covariance, k-1
arm_matrix_instance_f32 P_curr;			// A posteriori covariance, k
arm_matrix_instance_f32 P_minus;		// A priori covariance, k
arm_matrix_instance_f32 Q_prev;			// Process noise covariance, k-1


void init_processing(SensorData* IMU0_data, SensorData* IMU1_data) {
	uint16_t numRows, numCols; // temp vars

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
	arm_mat_init_f32(&w_avg_b0, numRows, numCols, w_avg_b0_f32);

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

	// Control matrix
	numRows = 12;
	numCols = 6;
	arm_mat_init_f32(&B_matrix, numRows, numCols, B_matrix_f32);

	// Input vector
	numRows = 6;
	numCols = 1;
	arm_mat_init_f32(&u_curr, numRows, numCols, u_curr_f32);

	// Observation matrix, swing phase
	numRows = 6;
	numCols = 12;
	arm_mat_init_f32(&H_swing, numRows, numCols, H_swing_f32);

	// Observation matrix, stance phase
	numRows = 12;
	numCols = 12;
	arm_mat_init_f32(&H_stance, numRows, numCols, H_stance_f32);

	// Observation vector, swing phase
	numRows = 6;
	numCols = 1;
	arm_mat_init_f32(&Z_swing, numRows, numCols, Z_swing_f32);

	// Observation vector, stance phase
	numRows = 12;
	numCols = 1;
	arm_mat_init_f32(&Z_stance, numRows, numCols, Z_stance_f32);

	// Kalman Gain, swing phase
	numRows = 12;
	numCols = 6;
	arm_mat_init_f32(&K_swing, numRows, numCols, K_swing_f32);

	// Kalman Gain, stance phase
	numRows = 12;
	numCols = 12;
	arm_mat_init_f32(&K_stance, numRows, numCols, K_stance_f32);

	// Observation noise covariance, swing phase
	numRows = 6;
	numCols = 6;
	arm_mat_init_f32(&R_swing, numRows, numCols, R_swing_f32);

	// Observation noise covariance, stance phase
	numRows = 12;
	numCols = 12;
	arm_mat_init_f32(&R_stance, numRows, numCols, R_stance_f32);

	// k-1 a posteriori covariance
	numRows = 12;
	numCols = 12;
	arm_mat_init_f32(&P_prev, numRows, numCols, P_prev_f32);

	// k a posteriori covariance
	numRows = 12;
	numCols = 12;
	arm_mat_init_f32(&P_curr, numRows, numCols, P_curr_f32);

	// k a priori covariance
	numRows = 12;
	numCols = 12;
	arm_mat_init_f32(&P_minus, numRows, numCols, P_minus_f32);

	// k-1 Process noise covariance
	numRows = 12;
	numCols = 12;
	arm_mat_init_f32(&Q_prev, numRows, numCols, Q_prev_f32);

	initZUPT(); // Initialize ZUPT phase detector

	initRingBuffers(IMU0_data, IMU1_data);

	initQuaternion(IMU0_data, IMU1_data);

}

void resetCurrentPosition(SensorData* IMU0_data, SensorData* IMU1_data) {

	int i;
	int j;
	for (i = 0; i < x_curr.numRows; ++i) {
		x_curr.pData[i] = x_init_f32[i];
	}

	for (i = 0; i < P_curr.numRows; ++i) {
		for (j = 0; j < P_curr.numCols; ++j) {
			P_curr.pData[(i*P_curr.numCols) + j] = P_init_f32[(i*P_curr.numCols) + j];
		}
	}

	updatePreviousMatrices();

	clearZUPT();
	initZUPT(); // Initialize ZUPT phase detector

	initRingBuffers(IMU0_data, IMU1_data);

	initQuaternion(IMU0_data, IMU1_data);
}

void calculateCorrectedState(
		SensorData* IMU0_data,
		SensorData* IMU1_data,
		float timeDelta) { // TODO Verify this

	enum PHASE phase;

	calculateAvgAngularRate(IMU0_data, IMU1_data); // w_avg_b0

	calculateRotationMatrix(timeDelta);	// R_b0_n

	updateFMatrix(timeDelta);	// Update F with new timeDelta

	updateBMatrix(timeDelta);	// Update B with new timeDelta

	updateUVector(IMU0_data, IMU1_data);	// Update u_curr with IMU data

	calculateStateEstimation();	// x(k) = F*x(k-1) + B*u(k)

	calculateStateEstimationErrorCovariance();	// P-(k) = F*P(k-1)*F^T + Q(k-1)
	// TODO need to tune process noise covariance matrix Q(k-1)

	//printf("%f", w_avg_b0_mag);

	phase = detectZUPTPhase();

	arm_matrix_instance_f32 Hi; // (Nx12)
	arm_matrix_instance_f32 Zi; // (Nx1)
	arm_matrix_instance_f32 Ri;	// (NxN) // TODO need to tune observation noise covariance matrix Ri(k)
	arm_matrix_instance_f32 Ki; // (12xN)

	if (phase == SWING) {
//		printf("Swing\n");
		// N = 6
		Hi = H_swing;
		Zi = Z_swing;
		Ri = R_swing;
		Ki = K_swing;
	} else { // phase == STANCE
//		printf("Stance\n");
		// N = 12
		Hi = H_stance;
		Zi = Z_stance;
		Ri = R_stance;
		Ki = K_stance;
	}
	// These matrices have different dimensions, so they need to be assigned to the instance of the correct size

	updateZiVector(&Zi);	// Update Observation Vector Z for optimal state estimation

	calculateGainMatrix(&Ki, &Hi, &Ri); // Ki(k) = P-(k)*Hi^T * (Hi*P-(k)*Hi^T + Ri(k))^-1

	calculateOptimalStateEstimation(&Ki, &Zi, &Hi);	// x(k) <-- x_best(k) = x(k) + Ki(k)*(Zi(k) - Hi*x(k))

	if (phase == STANCE) {
		x_curr_f32[2] = 0;
		x_curr_f32[5] = 0;
	}
	float v_gate = 10;
	if (fabs(x_curr_f32[8]) > v_gate) {
		x_curr_f32[8] = v_gate * (x_curr_f32[8] > 0 ? 1 : -1);
	}
	if (fabs(x_curr_f32[11]) > v_gate) {
		x_curr_f32[11] = v_gate * (x_curr_f32[11] > 0 ? 1 : -1);
	}

//	optimal_f32[0] = Z_stance_f32[0];
//	optimal_f32[1] = Z_stance_f32[4];
//	optimal_f32[2] = Z_stance_f32[5];
	optimal_f32[0] = (x_curr_f32[0]);
	optimal_f32[1] = (x_curr_f32[1] + x_curr_f32[4]) / 2;
	optimal_f32[2] = (x_curr_f32[2] + x_curr_f32[5]) / 2;

	calculateOptimalEstimationErrorCovariance(&Ki, &Hi, &Ri);	// P(k) = (I - Ki(k)*Hi)*P-(k)

	updatePreviousMatrices();	// update x_prev, P_prev, (Q_prev?) // TODO Add Q_prev to this?
	phase_out = phase;
}

float returnCurrentPosition(Position* current_pos) {

	current_pos->X = (x_curr_f32[0] + x_curr_f32[3]) / 2;
	current_pos->Y = (x_curr_f32[1] + x_curr_f32[4]) / 2;
	current_pos->Z = (x_curr_f32[2] + x_curr_f32[5]) / 2;
	// Returns avg of two position values, units of meters
	return 0.;
}

float returnDebugOutput(Position* corr, Position* pred, Position* optimal_pos, Position* K_gain, Position* w_avg, Quaternion* quat, Position* ZUPT) {
	corr->X = correction_f32[0];
	corr->Y = correction_f32[1];
	corr->Z = correction_f32[2];

	//*pred = (Position){0,0,0};
	pred->X = prediction_f32[0];
	pred->Y = prediction_f32[1];
	pred->Z = prediction_f32[2];

	optimal_pos->X = optimal_f32[0];
	optimal_pos->Y = optimal_f32[1];
	optimal_pos->Z = optimal_f32[2];

	K_gain->X = gain_f32[0];
	K_gain->Y = gain_f32[1];
	K_gain->Z = gain_f32[2];

	w_avg->X = w_avg_b0_f32[0];
	w_avg->Y = w_avg_b0_f32[1];
	w_avg->Z = w_avg_b0_f32[2];

	quat->W = q_f32[0];
	quat->X = q_f32[1];
	quat->Y = q_f32[2];
	quat->Z = q_f32[3];

	ZUPT->X = (float)curr_phase; // Phase
	ZUPT->Y = (float)phase_counter;

	return w_avg_b0_mag;
}

void calculateAvgAngularRate(
		SensorData* IMU0_data,
		SensorData* IMU1_data) {

	getNextGyroReading(IMU0_data, IMU1_data, w_avg_b0_f32);

	// Determine |w_avg_b0|
	w_avg_b0_mag = vec_mag_f32(w_avg_b0_f32);
}

void calculateRotationMatrix(
		float timeDelta) { // TODO Verify this

	// Determine change in rotation angle / 2 (units of radians)
	float rotation_angle_div_2 = w_avg_b0_mag * timeDelta * deg2rad / 2;

	float q1_3_scaling_term = (w_avg_b0_mag) ?
			(float)sin(rotation_angle_div_2) / w_avg_b0_mag : w_avg_b0_mag; // reduce number of calculations

	// Determine change in rotation as quaternion
	float delta_q_f32[4];
	delta_q_f32[0] = (float)cos(rotation_angle_div_2);
	delta_q_f32[1] = w_avg_b0_f32[0] * q1_3_scaling_term;
	delta_q_f32[2] = w_avg_b0_f32[1] * q1_3_scaling_term;
	delta_q_f32[3] = w_avg_b0_f32[2] * q1_3_scaling_term;

	arm_quaternion_normalize_f32(delta_q_f32, delta_q_f32, 1);	// q = q / |q|a

	// Calculate new normalized quaternion
	arm_quaternion_product_single_f32(delta_q_f32, q_f32, q_f32); // q = q x delta_q
	arm_quaternion_normalize_f32(q_f32, q_f32, 1);	// q = q / |q|
	// Calculate rotation matrix from board frame to nav frame using quaternion
	arm_quaternion2rotation_f32(q_f32, rotation_b0_n_f32, 1);
	//printf("%f %f %f %f %f %f %f %f %f\n", rotation_b0_n_f32[0], rotation_b0_n_f32[1], rotation_b0_n_f32[2], rotation_b0_n_f32[3], rotation_b0_n_f32[4], rotation_b0_n_f32[5], rotation_b0_n_f32[6], rotation_b0_n_f32[7], rotation_b0_n_f32[8]);
}

void calculateStateEstimation(void) { // TODO Verify this

	/*
	 *  Define Temporary Objects
	 */

	float temp1_f32[12];
	arm_matrix_instance_f32 temp1;
	arm_mat_init_f32(&temp1, 12, 1, temp1_f32); // temp for first term of sum, 12x1

	float temp2_f32[12];
	arm_matrix_instance_f32 temp2;
	arm_mat_init_f32(&temp2, 12, 1, temp2_f32); // temp for second term of sum, 12x1

	/*
	 *  Calculation Section
	 */

	arm_mat_mult_f32(&F_matrix, &x_prev, &temp1); // F*x(k-1) --> (12x12) * (12x1)

	arm_mat_mult_f32(&B_matrix, &u_curr, &temp2); // B*u(k) --> (12x6) * (6x1)

	arm_mat_add_f32(&temp1, &temp2, &x_curr); // x(k) = F*x(k-1) + B*u(k)

	prediction_f32[0] = (x_curr_f32[0]);
	prediction_f32[1] = (x_curr_f32[1] + x_curr_f32[4]) / 2;
	prediction_f32[2] = (x_curr_f32[2] + x_curr_f32[5]) / 2;
}

void calculateStateEstimationErrorCovariance(void) {

	/*
	 *  Define Temporary Objects
	 */

	float temp12x12_0_f32[144];
	arm_matrix_instance_f32 temp12x12_0;
	arm_mat_init_f32(&temp12x12_0, 12, 12, temp12x12_0_f32); // Temp 12x12 matrix
	arm_mat_trans_f32(&F_matrix, &temp12x12_0);	// Initialize to transpose of F

	float temp12x12_1_f32[144];
	arm_matrix_instance_f32 temp12x12_1;
	arm_mat_init_f32(&temp12x12_1, 12, 12, temp12x12_1_f32); // Temp 12x12 matrix
	/*
	 *  Calculation Section
	 */

	arm_mat_mult_f32(&P_prev, &temp12x12_0, &temp12x12_1);	// P(k-1)*F^T --> (12x12) * (12x12)

	arm_mat_mult_f32(&F_matrix, &temp12x12_1, &temp12x12_0);	// F*(P(k-1)*F^T) --> (12x12) * (12x12)

	arm_mat_add_f32(&temp12x12_0, &Q_prev, &P_minus);	// P-(k) = (F*P(k-1)*F^T) + Q(k-1)

}

#define MAX_IMU (4*g)

void calculateGainMatrix(
		arm_matrix_instance_f32* Ki, /*(12xN)*/
		arm_matrix_instance_f32* Hi, /*(Nx12)*/
		arm_matrix_instance_f32* Ri /*(NxN)*/) { // TODO Verify this

	uint16_t N = Hi->numRows;

	/*
	 *  Define Temporary Objects
	 */

	arm_matrix_instance_f32 temp12xN_0;
	arm_mat_init_f32(&temp12xN_0, 12, N, temp12xN_0_f32); // temp matrix (12xN)
	arm_mat_trans_f32(Hi, &temp12xN_0); // init to transpose of Hi

	arm_matrix_instance_f32 temp12xN_1;
	arm_mat_init_f32(&temp12xN_1, 12, N, temp12xN_1_f32); // temp matrix (12xN)

	arm_matrix_instance_f32 tempNxN_0;
	arm_mat_init_f32(&tempNxN_0, N, N, tempNxN_0_f32); // temp matrix (NxN)

	arm_matrix_instance_f32 tempNxN_1;
	arm_mat_init_f32(&tempNxN_1, N, N, tempNxN_1_f32); // temp matrix (NxN)

	arm_matrix_instance_f32 tempNxN_2;
	arm_mat_init_f32(&tempNxN_2, N, N, tempNxN_2_f32); // temp matrix (NxN)

	/*
	 *  Calculation Section
	 */

	float IMU[3];
	int index = ((int)xl_oldest - 1)%RING_SIZE;
	IMU[0] = xl0_avg_x_ring[index]/MAX_IMU;
	IMU[1] = xl0_avg_y_ring[index]/MAX_IMU;
	IMU[2] = xl0_avg_z_ring[index]/MAX_IMU;


	// Form Ri Scaling Matrix
	int i;
	for (i = 0; i < N; ++i) {
		int j;
		for (j = 0; j < 3; ++j) {
			if (((N+1)*i) % N == j) {
				tempNxN_0_f32[((N+1)*i)] = IMU[j] * dT;
			} else {
				tempNxN_0_f32[(i*N)+j] = 0;
			}
		}
		for (j = 3; j < 6; ++j) {
			if (((N+1)*i) % N == j) {
				tempNxN_0_f32[((N+1)*i)] = IMU[j%3] * dT2 / 2;
			} else {
				tempNxN_0_f32[(i*N)+j] = 0;
			}
		}
		for (j = 6; j < N; ++j) {
			tempNxN_0_f32[(i*N)+j] = 0;
		}
	}

	arm_mat_mult_f32(Ri, &tempNxN_0, &tempNxN_2);

	arm_mat_mult_f32(&P_minus, &temp12xN_0, &temp12xN_1);	// P-(k)*Hi^T --> (12x12) * (12xN)

	arm_mat_mult_f32(Hi, &temp12xN_1, &tempNxN_0);	// Hi*(P-(k)*Hi^T) --> (Nx12) * (12xN)

	arm_mat_add_f32(&tempNxN_0, &tempNxN_2, &tempNxN_1);	// (Hi*P-(k)*Hi^T + Ri(k))

	arm_mat_inverse_f32(&tempNxN_1, &tempNxN_0);	// (Hi*P-(k)*Hi^T + Ri(k))^-1

	arm_mat_mult_f32(&temp12xN_1, &tempNxN_0, Ki);	// Ki(k) = P-(k)*Hi^T * (Hi*P-(k)*Hi^T + Ri(k))^-1 --> (12xN) * (NxN)

	float k_gate = 5;
	for (i = 0; i < N*N; ++i) {
		if (fabs(Ki->pData[i]) > k_gate || Ki->pData != Ki->pData) {
			Ki->pData[i] = k_gate * (Ki->pData[i] > 0 ? 1 : -1);
			diverging_flag = 1;
		}
	}

	gain_f32[0] = (Ki->pData[0] + Ki->pData[3]) / 2;
	gain_f32[1] = (Ki->pData[1] + Ki->pData[4]) / 2;
	gain_f32[2] = (Ki->pData[2] + Ki->pData[5]) / 2;
}

#define X_THRES 0.1
#define Y_THRES 0.1
#define Z_THRES 0.1

void calculateOptimalStateEstimation(
		arm_matrix_instance_f32* Ki, /*(12xN)*/
		arm_matrix_instance_f32* Zi, /*(Nx1)*/
		arm_matrix_instance_f32* Hi /*(Nx12)*/) { // TODO Verify this

	uint16_t N = Zi->numRows;

	/*
	 *  Define Temporary Objects
	 */

	arm_matrix_instance_f32 tempNx1_0;
	arm_mat_init_f32(&tempNx1_0, N, 1, temp12x1_0_f32); // Will temporarily store some operation results, (Nx1)

	arm_matrix_instance_f32 tempNx1_1;
	arm_mat_init_f32(&tempNx1_1, N, 1, temp12x1_1_f32); // Will temporarily store some operation results, (Nx1)

	float temp12x1_f32[12];
	arm_matrix_instance_f32 temp12x1;
	arm_mat_init_f32(&temp12x1, 12, 1, temp12x1_f32); // Will temporarily store some operation results, (12x1)

	/*
	 *  Calculation Section
	 */

	arm_mat_mult_f32(Hi, &x_curr, &tempNx1_0);	// Hi*x(k) --> (Nx12) * (12x1)

	// Calculate correction factor
	arm_mat_sub_f32(Zi, &tempNx1_0, &tempNx1_1);	// (Zi(k) - Hi*x(k)) -> tempNx1
//	printf("%f %f %f\n", tempNx1_f32[3], tempNx1_f32[4], tempNx1_f32[5]); // should be ideally zero


	correction_f32[0] = (temp12x1_1_f32[0]); // Z - Hx
	// Weight correction factor by Kalman Gain
	arm_mat_mult_f32(Ki, &tempNx1_1, &temp12x1); // Ki(k) * (Zi(k) - Hi*x(k)) --> (12xN) * (Nx1) -> temp12x1

	correction_f32[1] = (temp12x1_f32[0]); // K(Z - Hx)

	//correction_f32[0] = (temp12x1_f32[0] + temp12x1_f32[3]) / 2;
	//correction_f32[1] = (temp12x1_f32[1] + temp12x1_f32[4]) / 2;
	correction_f32[2] = (temp12x1_f32[2] + temp12x1_f32[5]) / 2;

//	printf("X_curr_before: %f \n", (x_curr_f32[0] + x_curr_f32[3]) / 2);
//	printf("Correction: %f \n", (temp12x1_f32[0] + temp12x1_f32[3]) / 2);

	// Add weighted correction factor
	arm_mat_add_f32(&x_curr, &temp12x1, &x_curr); // x(k) <= x_best(k) = x(k) + Ki(k) * (Zi(k) - Hi*x(k))

//	printf("X_curr_after: %f \n", (x_curr_f32[0] + x_curr_f32[3]) / 2);
//	printf("~~~~~~~~~\n");

}

void calculateOptimalEstimationErrorCovariance(
		arm_matrix_instance_f32* Ki, /*(12xN)*/
		arm_matrix_instance_f32* Hi, /*(Nx12)*/
		arm_matrix_instance_f32* Ri /*(NxN)*/) { // TODO Verify this

	uint16_t N = Hi->numRows;

	/*
	 *  Define Temporary Objects
	 */

	arm_matrix_instance_f32 tempNx12_0;
	arm_mat_init_f32(&tempNx12_0, N, 12, tempNx12_0_f32); // temp matrix (Nx12)
	arm_mat_trans_f32(Ki, &tempNx12_0); // init to transpose of Ki

	arm_matrix_instance_f32 temp12xN_0;
	arm_mat_init_f32(&temp12xN_0, 12, N, temp12xN_0_f32); // temp matrix (12xN)

	float Identity12x12_f32[144] = {
			1,0,0,	0,0,0,	0,0,0,	0,0,0,
			0,1,0,	0,0,0,	0,0,0,	0,0,0,
			0,0,1,	0,0,0,	0,0,0,	0,0,0,

			0,0,0,	1,0,0,	0,0,0,	0,0,0,
			0,0,0,	0,1,0,	0,0,0,	0,0,0,
			0,0,0,	0,0,1,	0,0,0,	0,0,0,

			0,0,0,	0,0,0,	1,0,0,	0,0,0,
			0,0,0,	0,0,0,	0,1,0,	0,0,0,
			0,0,0,	0,0,0,	0,0,1,	0,0,0,

			0,0,0,	0,0,0,	0,0,0,	1,0,0,
			0,0,0,	0,0,0,	0,0,0,	0,1,0,
			0,0,0,	0,0,0,	0,0,0,	0,0,1,
	};
	arm_matrix_instance_f32 Identity12x12;
	arm_mat_init_f32(&Identity12x12, 12, 12, Identity12x12_f32); // 12x12 Identity matrix

	float temp12x12_0_f32[144];
	arm_matrix_instance_f32 temp12x12_0;
	arm_mat_init_f32(&temp12x12_0, 12, 12, temp12x12_0_f32); // Temp 12x12 matrix 0

	float temp12x12_1_f32[144];
	arm_matrix_instance_f32 temp12x12_1;
	arm_mat_init_f32(&temp12x12_1, 12, 12, temp12x12_1_f32); // Temp 12x12 matrix 1

	float temp12x12_2_f32[144];
	arm_matrix_instance_f32 temp12x12_2;
	arm_mat_init_f32(&temp12x12_2, 12, 12, temp12x12_2_f32); // Temp 12x12 matrix 2



	/*
	 *  Calculation Section
	 */

	// Check for divergence
	float rst_constant = 1;
	if (diverging_flag) {
		int i;
		int j;
		for (i = 0; i < P_curr.numRows; ++i) {
			for (j = 0; j < P_curr.numCols; ++j) {
				P_curr.pData[(i*P_curr.numCols) + j] = P_init_f32[(i*P_curr.numCols) + j] * rst_constant;
			}
		}
		diverging_flag = 0;
		return;
	}

	arm_mat_mult_f32(Ki, Hi, &temp12x12_0);	// Ki(k)*Hi --> (12xN) * (Nx12)

	arm_mat_sub_f32(&Identity12x12, &temp12x12_0, &temp12x12_1); // I - (Ki(k)*Hi)

	arm_mat_trans_f32(&temp12x12_1, &temp12x12_0); 	// (I - (Ki(k)*Hi)^T

	arm_mat_mult_f32(&temp12x12_1, &P_minus, &temp12x12_2);	// (I - Ki(k)*Hi)*P-(k) --> (12x12) * (12x12)

	arm_mat_mult_f32(&temp12x12_2, &temp12x12_0, &temp12x12_1);	// (I - Ki(k)*Hi)*P-(k)*(I - (Ki(k)*Hi)^T

	arm_mat_mult_f32(Ki, Ri, &temp12xN_0);	// Ki(k)*Ri --> (12xN) * (NxN)

	arm_mat_mult_f32(&temp12xN_0, &tempNx12_0, &temp12x12_0);	// Ki(k)*Ri*Ki(k)^T --> (12xN) * (Nx12)

	arm_mat_add_f32(&temp12x12_1, &temp12x12_0, &P_curr);	// P(k) = (I - Ki(k)*Hi)*P-(k)*(I - (Ki(k)*Hi)^T + Ki(k)*Ri*Ki(k)^T

}

void updateFMatrix(
		float timeDelta) {

	int i;
	for(i = 0; i < 6; ++i) { // Update specific indices of F matrix
		F_matrix_f32[6 + (13*i)] = timeDelta;
	}
}

void updateBMatrix(
		float timeDelta) { // TODO Verify this

	float dt2 = timeDelta * timeDelta / 2;

	int i;
	for(i = 0; i < 6; ++i) { // Update specific indices of B matrix with (timeDelta^2)/2
		B_matrix_f32[(7*i)] = dt2;
	}

	for(i = 0; i < 6; ++i) { // Update specific indices of B matrix with timeDelta
		B_matrix_f32[36 + (7*i)] = timeDelta;
	}
}

void updateUVector(
		SensorData* IMU0_data,
		SensorData* IMU1_data) {

	/*
	 *  Define Temporary Objects
	 */

	getNextXLReading(IMU0_data, IMU1_data, xl_b0_f32, xl_b1_f32);

	float temp0_f32[3] = {xl_b0_f32[0], xl_b0_f32[1], xl_b0_f32[2]}; // Init with IMU0 acceleration
	arm_matrix_instance_f32 temp0;
	arm_mat_init_f32(&temp0, 3, 1, temp0_f32); // temp for IMU0 vector

	float temp1_f32[3] = {xl_b1_f32[0], xl_b1_f32[1], xl_b1_f32[2]}; // Init with IMU1 acceleration
	arm_matrix_instance_f32 temp1;
	arm_mat_init_f32(&temp1, 3, 1, temp1_f32); // temp for IMU1 vector

	float temp2_f32[3] = {0,0,0};
	arm_matrix_instance_f32 temp2;
	arm_mat_init_f32(&temp2, 3, 1, temp2_f32); // temp for rotated IMU0 vector

	float temp3_f32[3] = {0,0,0};
	arm_matrix_instance_f32 temp3;
	arm_mat_init_f32(&temp3, 3, 1, temp3_f32); // temp for rotated IMU1 vector

	/*
	 *  Calculation Section
	 */
	//printf("%f %f %f %f %f %f %f %f %f\n", rotation_b0_n_f32[0], rotation_b0_n_f32[1], rotation_b0_n_f32[2], rotation_b0_n_f32[3], rotation_b0_n_f32[4], rotation_b0_n_f32[5], rotation_b0_n_f32[6], rotation_b0_n_f32[7], rotation_b0_n_f32[8]);


	//printf("%f %f %f \n", temp0_f32[0], temp0_f32[1], temp0_f32[2]);

	// Rotate IMU0 XL from board frame to nav frame
	arm_mat_mult_f32(&rotation_b0_n, &temp0, &temp2);	// R_b0_n*a0_b0 --> (3x3) * (3x1)

	//printf("%f %f %f \n", temp2_f32[0], temp2_f32[1], temp2_f32[2]);

	// Rotate IMU1 XL from board frame to nav frame
	arm_mat_mult_f32(&rotation_b0_n, &temp1, &temp3);	// R_b0_n*a1_b0 --> (3x3) * (3x1)

	// Subtract gravitation vector from IMU0 XL vector
	arm_mat_sub_f32(&temp2, &g_n, &temp0);
	//printf("%f %f %f \n", temp0_f32[0], temp0_f32[1], temp0_f32[2]);
	//printf("~~~~~~~~~~~~");

	// Subtract gravitation vector from IMU1 XL vector
	arm_mat_sub_f32(&temp3, &g_n, &temp1);


	/*
	 *  Update Section
	 */

	// Fill u input vector with IMU0 data
	u_curr_f32[0] = temp0_f32[0];
	u_curr_f32[1] = temp0_f32[1];
	u_curr_f32[2] = temp0_f32[2];

	// Fill u input vector with IMU1 data
	u_curr_f32[3] = temp1_f32[0];
	u_curr_f32[4] = temp1_f32[1];
	u_curr_f32[5] = temp1_f32[2];
}

void updateZiVector(
		arm_matrix_instance_f32* Zi) { // TODO Verify this

	/*
	 *  Define Temporary Objects
	 */

	float tempRm_f32[3];
	arm_matrix_instance_f32 tempRm;	// Rotation of m_b0 from b0 to n
	arm_mat_init_f32(&tempRm, 3, 1, tempRm_f32);
	float tempRw_f32[3];
	arm_matrix_instance_f32 tempRw;	// Rotation of w_avg_b0 from b0 to n
	arm_mat_init_f32(&tempRw, 3, 1, tempRw_f32);

	/*
	 *  Calculation Section
	 */

	// Rotate w_avg_b0 from board frame to nav frame
	arm_mat_mult_f32(&rotation_b0_n, &w_avg_b0, &tempRw);	// R_b0_n*w_avg_b0 --> (3x3) * (3x1)

	// Rotate m_b0 from board frame to nav frame
	arm_mat_mult_f32(&rotation_b0_n, &m_b0, &tempRm);	// R_b0_n*m_b0 --> (3x3) * (3x1)

	/*
	 *  Update Section
	 */

	// Copy Rm to second vector of Zi
	Zi->pData[3] = tempRm_f32[0];
	Zi->pData[4] = tempRm_f32[1];
	Zi->pData[5] = tempRm_f32[2];

	// Do (Rw x Rm), store in Rw as a temp location
	cross_product(&tempRw, &tempRm, &tempRw);

	// Copy (Rw x Rm) to first vector of Zi
	Zi->pData[0] = tempRw_f32[0];
	Zi->pData[1] = tempRw_f32[1];
	Zi->pData[2] = tempRw_f32[2];
}

void updatePreviousMatrices(void) {
	int i;
	int j;
	for (i = 0; i < x_curr.numRows; ++i) {
		x_prev.pData[i] = x_curr.pData[i];
	}

	for (i = 0; i < P_prev.numRows; ++i) {
		for (j = 0; j < P_prev.numCols; ++j) {
			P_prev.pData[(i*P_prev.numCols) + j] = P_curr.pData[(i*P_curr.numCols) + j];
		}
	}
}

float dot_f32(float* a, float* b) {
	return (a[0]*b[0]) + (a[1]*b[1]) + (a[2]*b[2]);
}

void cross_f32(float* a, float* b, float* c) {
	c[0] = a[1] * b[2] - a[2] * b[1];
	c[1] = a[2] * b[0] - a[0] * b[2];
	c[2] = a[0] * b[1] - a[1] * b[0];
}

float vec_mag_f32(float* vec) {
	float mag = (vec[0]*vec[0]) + (vec[1]*vec[1]) + (vec[2]*vec[2]);
	arm_sqrt_f32(mag, &mag);
	return mag;
}

void cross_product(
		arm_matrix_instance_f32* a,
		arm_matrix_instance_f32* b,
		arm_matrix_instance_f32* c) {

	// Make copy to avoid using overwritten data in calculations (ex. if a = c)
	float aData[3] = {a->pData[0], a->pData[1], a->pData[2]};
	float bData[3] = {b->pData[0], b->pData[1], b->pData[2]};

	cross_f32(aData, bData, c->pData);
}

void initRingBuffers(SensorData* IMU0_data, SensorData* IMU1_data) {
	float w_avg_tmp[3];
	w_avg_tmp[0] = (IMU0_data->G_X + IMU1_data->G_X) / 2;
	w_avg_tmp[1] = (IMU0_data->G_Y + IMU1_data->G_Y) / 2;
	w_avg_tmp[2] = (IMU0_data->G_Z + IMU1_data->G_Z) / 2;

	int i;
	for(i = 0; i < RING_SIZE; ++i) {
		w_avg_x_ring[i] = w_avg_tmp[0];
		w_avg_y_ring[i] = w_avg_tmp[1];
		w_avg_z_ring[i] = w_avg_tmp[2];
		xl0_avg_x_ring[i] = IMU0_data->XL_X;
		xl0_avg_y_ring[i] = IMU0_data->XL_Y;
		xl0_avg_z_ring[i] = IMU0_data->XL_Z;
		xl1_avg_x_ring[i] = IMU1_data->XL_X;
		xl1_avg_y_ring[i] = IMU1_data->XL_Y;
		xl1_avg_z_ring[i] = IMU1_data->XL_Z;
	}
}

void initQuaternion(SensorData* IMU0_data, SensorData* IMU1_data) {
	float avg_XL[3];
	avg_XL[0] = (IMU0_data->XL_X + IMU1_data->XL_X) / 2;
	avg_XL[1] = (IMU0_data->XL_Y + IMU1_data->XL_Y) / 2;
	avg_XL[2] = (IMU0_data->XL_Z + IMU1_data->XL_Z) / 2;

	float mag_avg_XL = vec_mag_f32(avg_XL);

	// Normalize average acceleration vector
	avg_XL[0] /= mag_avg_XL;
	avg_XL[1] /= mag_avg_XL;
	avg_XL[2] /= mag_avg_XL;

	float norm_g[3] = {0,0,1}; // Normalized vector for g_nav

	// q = [1+dot(r, r') cross(r, r') --> From conjugation: r' = q x [0 r] x q*

	float dot_prod = dot_f32(avg_XL, norm_g);
	float cross_prod[3];
	cross_f32(avg_XL, norm_g, cross_prod);

	if (dot_prod > 0.999999) {
		q_f32[0] = 1;
		q_f32[1] = 0;
		q_f32[2] = 0;
		q_f32[3] = 0;

		return;

	} else if (dot_prod < -0.999999) {
		float xUnit[3] = {1,0,0};
		float yUnit[3] = {0,1,0};
		float tempVec[3];
        cross_f32(xUnit, avg_XL, tempVec);

        float tempVecMag = vec_mag_f32(tempVec);

        if (tempVecMag < 0.000001) {
            cross_f32(yUnit, avg_XL, tempVec);
			tempVecMag = vec_mag_f32(tempVec);
		}

        q_f32[0] = 0;
        q_f32[1] = tempVec[0] / tempVecMag;
        q_f32[2] = tempVec[1] / tempVecMag;
        q_f32[3] = tempVec[2] / tempVecMag;
        return;
    }

	q_f32[0] = 1 + dot_prod;
	q_f32[1] = cross_prod[0];
	q_f32[2] = cross_prod[1];
	q_f32[3] = cross_prod[2];

	arm_quaternion_normalize_f32(q_f32, q_f32, 1); // normalize initial quaternion

}

void getNextGyroReading(SensorData* IMU0_data, SensorData* IMU1_data, float* gyroOut) {
	w_avg_x_ring[w_oldest] = (IMU0_data->G_X + IMU1_data->G_X) / 2;
	w_avg_y_ring[w_oldest] = (IMU0_data->G_Y + IMU1_data->G_Y) / 2;
	w_avg_z_ring[w_oldest] = (IMU0_data->G_Z + IMU1_data->G_Z) / 2;

	w_oldest = (w_oldest + 1) % RING_SIZE;

	gyroOut[0] = 0;
	gyroOut[1] = 0;
	gyroOut[2] = 0;

	int i;
	for(i = 0; i < RING_SIZE; ++i) {
		gyroOut[0] += w_avg_x_ring[i];
		gyroOut[1] += w_avg_y_ring[i];
		gyroOut[2] += w_avg_z_ring[i];
	}

	gyroOut[0] /= RING_SIZE;
	gyroOut[1] /= RING_SIZE;
	gyroOut[2] /= RING_SIZE;
}

void getNextXLReading(SensorData* IMU0_data, SensorData* IMU1_data, float* xl0Out, float* xl1Out) {
	xl0_avg_x_ring[xl_oldest] = IMU0_data->XL_X;
	xl0_avg_y_ring[xl_oldest] = IMU0_data->XL_Y;
	xl0_avg_z_ring[xl_oldest] = IMU0_data->XL_Z;
	xl1_avg_x_ring[xl_oldest] = IMU1_data->XL_X;
	xl1_avg_y_ring[xl_oldest] = IMU1_data->XL_Y;
	xl1_avg_z_ring[xl_oldest] = IMU1_data->XL_Z;

	xl_oldest = (xl_oldest + 1) % RING_SIZE;

	xl0Out[0] = 0;
	xl0Out[1] = 0;
	xl0Out[2] = 0;
	xl1Out[0] = 0;
	xl1Out[1] = 0;
	xl1Out[2] = 0;

	int i;
	for(i = 0; i < RING_SIZE; ++i) {
		xl0Out[0] += xl0_avg_x_ring[i];
		xl0Out[1] += xl0_avg_y_ring[i];
		xl0Out[2] += xl0_avg_z_ring[i];
		xl1Out[0] += xl1_avg_x_ring[i];
		xl1Out[1] += xl1_avg_y_ring[i];
		xl1Out[2] += xl1_avg_z_ring[i];
	}

	xl0Out[0] /= RING_SIZE;
	xl0Out[1] /= RING_SIZE;
	xl0Out[2] /= RING_SIZE;
	xl1Out[0] /= RING_SIZE;
	xl1Out[1] /= RING_SIZE;
	xl1Out[2] /= RING_SIZE;
}

void initZUPT(void) {
	ZUPTHead = (ZUPTNode*)createZUPTNode(0.0);
	ZUPTNode* tempNode = ZUPTHead;

	int i;
	for(i = 0; i < ZUPT_W-1; ++i) {
		tempNode->next = (ZUPTNode*)createZUPTNode(0.0);
		tempNode = (ZUPTNode*)tempNode->next;
	}
}

void clearZUPT(void) {
	ZUPTNode* tempNode = ZUPTHead;

	curr_phase = STANCE;
	phase_counter = 0;

	if (tempNode == NULL) {
		return;
	}

	while (tempNode->next != NULL) {
		ZUPTNode* tempNode2 = tempNode->next;

		free(tempNode);

		tempNode = tempNode2;
	}
}

enum PHASE detectZUPTPhase(void) {
	/*
	 *  1) Move head to next, free head
	 *  2) Sum over W-1 nodes
	 *  3) Append new node to end of list and add to sum
	 *  4) Scale by 1/(sigma^2 * W) --> Tw
	 *  5) Compare Tw to threshold, return stance if less than AND interval reqs are met
	 */

	// 1) Move head to next, free head
	assert(ZUPTHead != NULL);
	ZUPTNode* tempNode = (ZUPTNode*)ZUPTHead->next;
	free(ZUPTHead);
	ZUPTHead = tempNode;

	// 2)Sum over W-1 nodes
	float sum = 0;
	int i;
	for(i = 0; i < ZUPT_W-2; ++i) {
		assert(tempNode != NULL);
		sum += tempNode->w_mag_sq;
		tempNode = (ZUPTNode*)tempNode->next;
	}
	assert(tempNode != NULL);
	sum += tempNode->w_mag_sq; // last node

	// 3) Append new node to end of list and add to sum
	tempNode->next = (ZUPTNode*)createZUPTNode(w_avg_b0_mag);
	assert(tempNode->next != NULL);
	tempNode = (ZUPTNode*)tempNode->next;
	sum += tempNode->w_mag_sq;

	// 4) Scale by 1/(sigma^2 * W) --> Tw
	float Tw = sum * ZUPT_SCALE_FACTOR;

	//printf("%f ", Tw);

	// 5) Compare Tw to threshold, return stance if less than AND interval reqs are met

	// Check detected phase against saturating counter
	enum PHASE detected_phase = (Tw < ZUPT_THRESHOLD) ? STANCE : SWING;
	if (detected_phase != curr_phase) {
		if (detected_phase == SWING) {
			// Move counter in swing direction
			++phase_counter;

			// If counter saturated, switch phase to swing
			curr_phase = (phase_counter == PHASE_INTERVAL_THRESHOLD) ? SWING : STANCE;
		} else { // STANCE
			// Move counter in stance direction
			--phase_counter;

			// If counter saturated, switch phase to stance
			curr_phase = (phase_counter == 0) ? STANCE : SWING;
		}
	}

	return curr_phase;
}

ZUPTNode* createZUPTNode(float w_mag) {
	ZUPTNode* node = (ZUPTNode*)malloc(sizeof(ZUPTNode));

	node->next = NULL;
	node->w_mag_sq = w_mag * w_mag;

	return node;
}
