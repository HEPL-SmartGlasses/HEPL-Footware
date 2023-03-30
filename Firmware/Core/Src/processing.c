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
float m_b0_f32[3] = { // (3x1)
		0.0127,
		-0.0127,
		0,
}; // Relative position of IMUs in board frame (b0, so relative to IMU0), unit (m)

float g_n_f32[3] = { // (3x1)
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
		1.9612, -0.0049, -0.2758, 0,
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

		0.0127,
		-0.0127,
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

		0.0127,
		-0.0127,
		0,

		0,
		0,
		0,

		0,
		0,
		0,
}; // State variable at k, x = [r0_n, r1_n, v0_n, v1_n]^T, init to all zeros

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

float H_swing_f32[72] = { // (6x12)
		0,0,0, 0,0,0, -1,0,0, 1,0,0,
		0,0,0, 0,0,0, 0,-1,0, 0,1,0,
		0,0,0, 0,0,0, 0,0,-1, 0,0,1,

		-1,0,0, 1,0,0, 0,0,0, 0,0,0,
		0,-1,0, 0,1,0, 0,0,0, 0,0,0,
		0,0,-1, 0,0,1, 0,0,0, 0,0,0,
}; // Observation matrix H_1, swing phase

float H_stance_f32[144] = { // (12x12)
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
}; // Observation matrix H_1, swing phase, init to all zeros

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
}; // Observation matrix H_2, stance phase, init to all zeros (last 2 vectors stay as zero)

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

#define dT 0.005
#define dT2 dT*dT
#define dT3 dT2*dT
#define r_tun 2.4 * g / 1000 * dT
#define r_tune 5

float R_swing_f32[36] = { // (6x6)
		r_tune,0,0,	0,0,0,
		0,r_tune,0,	0,0,0,
		0,0,r_tune,	0,0,0,

		0,0,0,		r_tune,0,0,
		0,0,0,		0,r_tune,0,
		0,0,0,		0,0,r_tune,
}; // Observation noise covariance, swing phase, init to // TODO fill this in

float R_stance_f32[144] = { // (12x12)
		r_tune,0,0,	0,0,0,	0,0,0,	0,0,0,
		0,r_tune,0,	0,0,0,	0,0,0,	0,0,0,
		0,0,r_tune,	0,0,0,	0,0,0,	0,0,0,

		0,0,0,			r_tune,0,0,	0,0,0,	0,0,0,
		0,0,0,			0,r_tune,0,	0,0,0,	0,0,0,
		0,0,0,			0,0,r_tune,	0,0,0,	0,0,0,

		0,0,0,	0,0,0,	r_tune,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,r_tune,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,r_tune,	0,0,0,

		0,0,0,	0,0,0,	0,0,0,	r_tune,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,r_tune,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,r_tune,
}; // Observation noise covariance, stance phase, init to // TODO fill this in

float P_prev_f32[] = { // (12x12)
		0,0,0,	0,0,0,	0,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,0,

		0,0,0,	0,0,0,	0,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,0,

		0,0,0,	0,0,0,	16,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,16,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,16,	0,0,0,

		0,0,0,	0,0,0,	0,0,0,	16,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,16,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,16,
}; // A posteriori covariance, k-1

float P_curr_f32[] = { // (12x12)
		0,0,0,	0,0,0,	0,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,0,

		0,0,0,	0,0,0,	0,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,0,

		0,0,0,	0,0,0,	16,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,16,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,16,	0,0,0,

		0,0,0,	0,0,0,	0,0,0,	16,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,16,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,16,
}; // A posteriori covariance, k

float P_minus_f32[] = { // (12x12)
		0,0,0,	0,0,0,	0,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,0,

		0,0,0,	0,0,0,	0,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,0,

		0,0,0,	0,0,0,	16,0,0,	0,0,0,
		0,0,0,	0,0,0,	0,16,0,	0,0,0,
		0,0,0,	0,0,0,	0,0,16,	0,0,0,

		0,0,0,	0,0,0,	0,0,0,	16,0,0,
		0,0,0,	0,0,0,	0,0,0,	0,16,0,
		0,0,0,	0,0,0,	0,0,0,	0,0,16,
}; // A priori covariance, k

float Q_prev_f32[] = { // (12x12)
		4*dT3,4*dT3,4*dT3,	0,0,0,	6*dT2,6*dT2,6*dT2,	0,0,0,
		4*dT3,4*dT3,4*dT3,	0,0,0,	6*dT2,6*dT2,6*dT2,	0,0,0,
		4*dT3,4*dT3,4*dT3,	0,0,0,	6*dT2,6*dT2,6*dT2,	0,0,0,

		0,0,0,	4*dT3,4*dT3,4*dT3,	0,0,0,	6*dT2,6*dT2,6*dT2,
		0,0,0,	4*dT3,4*dT3,4*dT3,	0,0,0,	6*dT2,6*dT2,6*dT2,
		0,0,0,	4*dT3,4*dT3,4*dT3,	0,0,0,	6*dT2,6*dT2,6*dT2,

		6*dT2,6*dT2,6*dT2,	0,0,0,	12*dT,12*dT,12*dT,	0,0,0,
		6*dT2,6*dT2,6*dT2,	0,0,0,	12*dT,12*dT,12*dT,	0,0,0,
		6*dT2,6*dT2,6*dT2,	0,0,0,	12*dT,12*dT,12*dT,	0,0,0,

		0,0,0,	6*dT2,6*dT2,6*dT2,	0,0,0,	12*dT,12*dT,12*dT,
		0,0,0,	6*dT2,6*dT2,6*dT2,	0,0,0,	12*dT,12*dT,12*dT,
		0,0,0,	6*dT2,6*dT2,6*dT2,	0,0,0,	12*dT,12*dT,12*dT,
}; // Process noise covariance, k-1, init to // TODO fill this in

/*
 *  Debug arrays
 */
float phase_out = 0;
float optimal_f32[3] = {0,0,0,};	// x, y, z
float correction_f32[3] = {0,0,0,}; // x, y, z
float prediction_f32[3] = {0,0,0,}; // x, y, z
float gain_f32[3] = {0,0,0,}; // x, y, z

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


void init_processing(void) {
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

	initRingBuffers();

	arm_quaternion_normalize_f32(q_f32, q_f32, 1); // normalize initial quaternion

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
	float v_gate = 0.3;
	if (fabs(x_curr_f32[8]) > v_gate) {
		x_curr_f32[8] = v_gate * (x_curr_f32[8] > 0 ? 1 : -1);
	}
	if (fabs(x_curr_f32[11]) > v_gate) {
		x_curr_f32[11] = v_gate * (x_curr_f32[11] > 0 ? 1 : -1);
	}

	optimal_f32[0] = (x_curr_f32[0] + x_curr_f32[3]) / 2;
	optimal_f32[1] = (x_curr_f32[1] + x_curr_f32[4]) / 2;
	optimal_f32[2] = (x_curr_f32[2] + x_curr_f32[5]) / 2;

	calculateOptimalEstimationErrorCovariance(&Ki, &Hi);	// P(k) = (I - Ki(k)*Hi)*P-(k)

	updatePreviousMatrices();	// update x_prev, P_prev, (Q_prev?) // TODO Add Q_prev to this?
	phase_out = (float)phase;
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

	*pred = (Position){0,0,0};
//	pred->X = prediction_f32[0];
//	pred->Y = prediction_f32[1];
//	pred->Z = prediction_f32[2];

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

	ZUPT->X = phase_out; // Phase mapped to X

	return w_avg_b0_mag;
}

void calculateAvgAngularRate(
		SensorData* IMU0_data,
		SensorData* IMU1_data) { // TODO Verify this

	getNextGyroReading(IMU0_data, IMU1_data, w_avg_b0_f32);

	// Determine |w_avg_b0|
	w_avg_b0_mag = (w_avg_b0_f32[0]*w_avg_b0_f32[0]) + (w_avg_b0_f32[1]*w_avg_b0_f32[1]) + (w_avg_b0_f32[2]*w_avg_b0_f32[2]);
	arm_sqrt_f32(w_avg_b0_mag, &w_avg_b0_mag);
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

	arm_quaternion_normalize_f32(delta_q_f32, delta_q_f32, 1);	// q = q / |q|

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

	prediction_f32[0] = (x_curr_f32[0] + x_curr_f32[3]) / 2;
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

void calculateGainMatrix(
		arm_matrix_instance_f32* Ki, /*(12xN)*/
		arm_matrix_instance_f32* Hi, /*(Nx12)*/
		arm_matrix_instance_f32* Ri /*(NxN)*/) { // TODO Verify this

	uint16_t N = Hi->numRows;

	/*
	 *  Define Temporary Objects
	 */

	float* temp12xN_0_f32 = (float*)malloc(12 * N * sizeof(float));
	arm_matrix_instance_f32 temp12xN_0;
	arm_mat_init_f32(&temp12xN_0, 12, N, temp12xN_0_f32); // temp matrix (12xN)
	arm_mat_trans_f32(Hi, &temp12xN_0); // init to transpose of Hi

	float* temp12xN_1_f32 = (float*)malloc(12 * N * sizeof(float));
	arm_matrix_instance_f32 temp12xN_1;
	arm_mat_init_f32(&temp12xN_1, 12, N, temp12xN_1_f32); // temp matrix (12xN)

	float* tempNxN_0_f32 = (float*)malloc(N * N * sizeof(float));
	arm_matrix_instance_f32 tempNxN_0;
	arm_mat_init_f32(&tempNxN_0, N, N, tempNxN_0_f32); // temp matrix (NxN)

	float* tempNxN_1_f32 = (float*)malloc(N * N * sizeof(float));
	arm_matrix_instance_f32 tempNxN_1;
	arm_mat_init_f32(&tempNxN_1, N, N, tempNxN_1_f32); // temp matrix (NxN)

	/*
	 *  Calculation Section
	 */

	arm_mat_mult_f32(&P_minus, &temp12xN_0, &temp12xN_1);	// P-(k)*Hi^T --> (12x12) * (12xN)

	arm_mat_mult_f32(Hi, &temp12xN_1, &tempNxN_0);	// Hi*(P-(k)*Hi^T) --> (Nx12) * (12xN)

	arm_mat_add_f32(&tempNxN_0, Ri, &tempNxN_1);	// (Hi*P-(k)*Hi^T + Ri(k))

	arm_mat_inverse_f32(&tempNxN_1, &tempNxN_0);	// (Hi*P-(k)*Hi^T + Ri(k))^-1

	arm_mat_mult_f32(&temp12xN_1, &tempNxN_0, Ki);	// Ki(k) = P-(k)*Hi^T * (Hi*P-(k)*Hi^T + Ri(k))^-1 --> (12xN) * (NxN)

	/*
	 *  Cleanup Section
	 */

	// Free malloc'd memory
	free(temp12xN_0_f32);
	free(tempNxN_0_f32);
	free(temp12xN_1_f32);
	free(tempNxN_1_f32);

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

	float* tempNx1_0_f32 = (float*)malloc(N * sizeof(float));
	arm_matrix_instance_f32 tempNx1_0;
	arm_mat_init_f32(&tempNx1_0, N, 1, tempNx1_0_f32); // Will temporarily store some operation results, (Nx1)

	float* tempNx1_1_f32 = (float*)malloc(N * sizeof(float));
	arm_matrix_instance_f32 tempNx1_1;
	arm_mat_init_f32(&tempNx1_1, N, 1, tempNx1_1_f32); // Will temporarily store some operation results, (Nx1)

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

	// Weight correction factor by Kalman Gain
	arm_mat_mult_f32(Ki, &tempNx1_1, &temp12x1); // Ki(k) * (Zi(k) - Hi*x(k)) --> (12xN) * (Nx1) -> temp12x1

	correction_f32[0] = (temp12x1_f32[0] + temp12x1_f32[3]) / 2;
	correction_f32[1] = (temp12x1_f32[1] + temp12x1_f32[4]) / 2;
	correction_f32[2] = (temp12x1_f32[2] + temp12x1_f32[5]) / 2;

//	printf("X_curr_before: %f \n", (x_curr_f32[0] + x_curr_f32[3]) / 2);
//	printf("Correction: %f \n", (temp12x1_f32[0] + temp12x1_f32[3]) / 2);

	// Add weighted correction factor
	arm_mat_add_f32(&x_curr, &temp12x1, &x_curr); // x(k) <= x_best(k) = x(k) + Ki(k) * (Zi(k) - Hi*x(k))

//	printf("X_curr_after: %f \n", (x_curr_f32[0] + x_curr_f32[3]) / 2);
//	printf("~~~~~~~~~\n");

	/*
	 *  Cleanup Section
	 */

	// Free malloc'd memory
	free(tempNx1_0_f32);
	free(tempNx1_1_f32);
}

void calculateOptimalEstimationErrorCovariance(
		arm_matrix_instance_f32* Ki, /*(12xN)*/
		arm_matrix_instance_f32* Hi /*(Nx12)*/) { // TODO Verify this

	/*
	 *  Define Temporary Objects
	 */

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

	float temp12x12_f32[144];
	arm_matrix_instance_f32 temp12x12;
	arm_mat_init_f32(&temp12x12, 12, 12, temp12x12_f32); // Temp 12x12 matrix

	/*
	 *  Calculation Section
	 */

	arm_mat_mult_f32(Ki, Hi, &temp12x12);	// Ki(k)*Hi --> (12xN) * (Nx12)

	arm_mat_sub_f32(&Identity12x12, &temp12x12, &temp12x12); // I - (Ki(k)*Hi)

	arm_mat_mult_f32(&temp12x12, &P_minus, &P_curr);	// P(k) = (I - Ki(k)*Hi)*P-(k) --> (12x12) * (12x12)

}

void updateFMatrix(
		float timeDelta) { // TODO Verify this

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
		SensorData* IMU1_data) { // TODO Verify this

	/*
	 *  Define Temporary Objects
	 */

	float temp0_f32[3] = {IMU0_data->XL_X, IMU0_data->XL_Y, IMU0_data->XL_Z}; // Init with IMU0 acceleration
	arm_matrix_instance_f32 temp0;
	arm_mat_init_f32(&temp0, 3, 1, temp0_f32); // temp for IMU0 vector

	float temp1_f32[3] = {IMU1_data->XL_X, IMU1_data->XL_Y, IMU1_data->XL_Z}; // Init with IMU1 acceleration
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

	getNextXLReading(IMU0_data, IMU1_data, xl_b0_f32, xl_b1_f32);

	u_curr_f32[0] = xl_b0_f32[0];
	u_curr_f32[1] = xl_b0_f32[1];
	u_curr_f32[2] = xl_b0_f32[2];

	// Fill u input vector with IMU1 data
	u_curr_f32[3] = xl_b1_f32[0];
	u_curr_f32[4] = xl_b1_f32[1];
	u_curr_f32[5] = xl_b1_f32[2];
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

void updatePreviousMatrices(void) { // TODO Verify this
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

	// TODO Update Q(k-1) somehow
}

void cross_product(
		arm_matrix_instance_f32* a,
		arm_matrix_instance_f32* b,
		arm_matrix_instance_f32* c) { // TODO Verify this

	// Make copy to avoid using overwritten data in calculations (ex. if a = c)
	float aData[3] = {a->pData[0], a->pData[1], a->pData[2]};
	float bData[3] = {b->pData[0], b->pData[1], b->pData[2]};

	c->pData[0] = aData[1] * bData[2] - aData[2] * bData[1];
	c->pData[1] = aData[2] * bData[0] - aData[0] * bData[2];
	c->pData[2] = aData[0] * bData[1] - aData[1] * bData[0];
}

void initRingBuffers(void) {
	int i;
	for(i = 0; i < RING_SIZE; ++i) {
		w_avg_x_ring[i] = 0;
		w_avg_y_ring[i] = 0;
		w_avg_z_ring[i] = 0;
		xl0_avg_x_ring[i] = 0;
		xl0_avg_y_ring[i] = 0;
		xl0_avg_z_ring[i] = 0;
		xl1_avg_x_ring[i] = 0;
		xl1_avg_y_ring[i] = 0;
		xl1_avg_z_ring[i] = 0;
	}
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
		tempNode->next = (struct ZUPTNode*)createZUPTNode(0.0);
		tempNode = (ZUPTNode*)tempNode->next;
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

	// Saturating interval counter
	static enum PHASE curr_phase = STANCE;
	static uint8_t phase_counter = 0; // 0 --> stance side, PHASE_INTERVAL_THRESHOLD --> swing side

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
	tempNode->next = (struct ZUPTNode*)createZUPTNode(w_avg_b0_mag);
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

struct ZUPTNode* createZUPTNode(float w_mag) {
	ZUPTNode* node = (ZUPTNode*)malloc(sizeof(ZUPTNode));

	node->next = NULL;
	node->w_mag_sq = w_mag * w_mag;
}
