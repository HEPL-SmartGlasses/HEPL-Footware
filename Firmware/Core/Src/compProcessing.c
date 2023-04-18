/*
 * compProcessing.c
 *
 *  Created on: Apr 17, 2023
 *      Author: evanm
 */

#include "compProcessing.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"

ZUPTNode* ZUPTHead; // Head to ZUPT linked list

float w_avg_b0_mag;

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

float q_gyro_f32[4] = { // (4x1)
		1,
		0,
		0,
		0,
}; // Quaternion representing rotation of board frame from nav frame, init to Identity quaternion (1, 0, 0, 0)

float q_accel_f32[4] = { // (4x1)
		1,
		0,
		0,
		0,
}; // Quaternion representing rotation of board frame from nav frame, init to Identity quaternion (1, 0, 0, 0)

float q_true_f32[4] = { // (4x1)
		1,
		0,
		0,
		0,
}; // Quaternion representing rotation of board frame from nav frame, init to Identity quaternion (1, 0, 0, 0)

float w_avg_b0_f32[3] = { // (3x1)
		0,
		0,
		0,
}; // Average angular rate vector of b0 relative to nav in b0, unit (deg/s)

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

void init_comp_processing(SensorData* IMU0_data, SensorData* IMU1_data) {
	int numRows;
	int numCols;

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

	initZUPT(); // Initialize ZUPT phase detector

	initRingBuffers(IMU0_data, IMU1_data);

	initQuaternions(IMU0_data, IMU1_data);
}

void calculateCompCorrectedState(SensorData* IMU0_data, SensorData* IMU1_data, float timeDelta) {

	enum PHASE phase;

	updateFMatrix(timeDelta);	// Update F with new timeDelta

	updateBMatrix(timeDelta);	// Update B with new timeDelta

	calculateRotationMatrix();	// R_b0_n

	updateUVector(IMU0_data, IMU1_data);	// Update u_curr with IMU data

	calculateAvgAngularRate(IMU0_data, IMU1_data); // w_avg_b0

	updateGyroQuat(timeDelta);

	updateAccelQuat(timeDelta);

	phase = detectZUPTPhase();

	float alpha = 0.9;
	quatSLERP(q_accel_f32, q_gyro_f32, q_true_f32, alpha); // Weighted interpolation between qX, qG --> qT

	calculateStateEstimation();	// x(k) = F*x(k-1) + B*u(k)

	const float AGGRESSIVE_ZUPT = 1;

	if (phase == STANCE) {
		// Zero z position
		x_curr_f32[2] = 0;
		x_curr_f32[5] = 0;

		// Zero velocity constraint
		int i;
		for (i = 6; i < 12; ++i) {
			x_curr_f32[i] = 0;
		}

		if (AGGRESSIVE_ZUPT) {
			// No X drift allowed >:)
			x_curr_f32[0] = x_prev_f32[0];
			x_curr_f32[1] = x_prev_f32[1];
			x_curr_f32[3] = x_prev_f32[3];
			x_curr_f32[4] = x_prev_f32[4];
		}
	}

	updatePreviousMatrices();	// update x_prev, P_prev, (Q_prev?) // TODO Add Q_prev to this?
	phase_out = phase;

}

void euler_to_quaternion(float* XL_angles, float* quat) {

	float yaw = XL_angles[0];
	float pitch = XL_angles[1];
	float roll = XL_angles[2];

	// Determine change in rotation as quaternion
	float delta_q_f32[4];
	delta_q_f32[0] = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);	// qw

	delta_q_f32[1] = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);	// qx

	delta_q_f32[2] = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);	// qy

	delta_q_f32[3] = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);	// qz

	arm_quaternion_normalize_f32(delta_q_f32, delta_q_f32, 1);	// q = q / |q|a

	// Shallow copy of prev q_gyro
	float temp_q_f32[4];
	temp_q_f32[0] = q_true_f32[0];
	temp_q_f32[1] = q_true_f32[1];
	temp_q_f32[2] = q_true_f32[2];
	temp_q_f32[3] = q_true_f32[3];

	// Calculate new normalized quaternion
	arm_quaternion_product_single_f32(temp_q_f32, delta_q_f32, quat); // q = q x delta_q
	arm_quaternion_normalize_f32(quat, quat, 1);	// q = q / |q|

}

void calculateAvgAngularRate(
		SensorData* IMU0_data,
		SensorData* IMU1_data) {

	getNextGyroReading(IMU0_data, IMU1_data, w_avg_b0_f32);

	// Determine |w_avg_b0|
	w_avg_b0_mag = vec_mag_f32(w_avg_b0_f32);
}

void getAccelAngles(float* XL_angles) {

	// Uses u_curr --> Nav. Frame Accelerations

	// XL_angles: yaw, pitch, roll

	float ax = (u_curr_f32[0] + u_curr_f32[3]) / 2;
	float ay = (u_curr_f32[1] + u_curr_f32[4]) / 2;
	float az = (u_curr_f32[2] + u_curr_f32[5]) / 2;

	float pitch = asin(ax / g);
	float roll = atan(ay / az);

	XL_angles[0] = 0;
	XL_angles[1] = pitch;
	XL_angles[2] = roll;
}

void quatSLERP(float* q1, float* q2, float* q3, float t) {
	float dot_prod = dot_f32(q1, q2) + (q1[3]*q2[3]);

	float angle = acos(dot_prod);
	float denom = sin(angle);

	if (dot_prod > 0.99999) {
		q3[0] = q2[0];
		q3[1] = q2[1];
		q3[2] = q2[2];
		q3[3] = q2[3];
	} else {
		q3[0] = (q1[0]*sin((1-t)*angle) + q2[0]*sin(t*angle)) / denom;
		q3[1] = (q1[1]*sin((1-t)*angle) + q2[1]*sin(t*angle)) / denom;
		q3[2] = (q1[2]*sin((1-t)*angle) + q2[2]*sin(t*angle)) / denom;
		q3[3] = (q1[3]*sin((1-t)*angle) + q2[3]*sin(t*angle)) / denom;
	}

	arm_quaternion_normalize_f32(q3, q3, 1);
}

void updateGyroQuat(float timeDelta) {
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

	// Shallow copy of prev q_gyro
	float temp_q_f32[4];
	temp_q_f32[0] = q_true_f32[0];
	temp_q_f32[1] = q_true_f32[1];
	temp_q_f32[2] = q_true_f32[2];
	temp_q_f32[3] = q_true_f32[3];

	// Calculate new normalized quaternion
	arm_quaternion_product_single_f32(temp_q_f32, delta_q_f32, q_gyro_f32); // q = q x delta_q
	arm_quaternion_normalize_f32(q_gyro_f32, q_gyro_f32, 1);	// q = q / |q|
}

void updateAccelQuat(float timeDelta) {
	float XL_angles[3];

	getAccelAngles(XL_angles);

	euler_to_quaternion(XL_angles, q_accel_f32);
}

void calculateRotationMatrix(void) {

	// Calculate rotation matrix from board frame to nav frame using quaternion
	arm_quaternion2rotation_f32(q_true_f32, rotation_b0_n_f32, 1);

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

void initQuaternions(SensorData* IMU0_data, SensorData* IMU1_data) {
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

	float q_f32[4];

	// q = [1+dot(r, r') cross(r, r') --> From conjugation: r' = q x [0 r] x q*

	float dot_prod = dot_f32(avg_XL, norm_g);
	float cross_prod[3];
	cross_f32(avg_XL, norm_g, cross_prod);

	if (dot_prod > 0.999999) {
		q_f32[0] = 1;
		q_f32[1] = 0;
		q_f32[2] = 0;
		q_f32[3] = 0;

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

    } else {

		q_f32[0] = 1 + dot_prod;
		q_f32[1] = cross_prod[0];
		q_f32[2] = cross_prod[1];
		q_f32[3] = cross_prod[2];

    }

	arm_quaternion_normalize_f32(q_f32, q_f32, 1); // normalize initial quaternion

	int i;
	for(i = 0; i < 4; ++i) {
		q_gyro_f32[i] = q_f32[i];
		q_accel_f32[i] = q_f32[i];
		q_true_f32[i] = q_f32[i];
	}

}

float returnCompDebugOutput(Position* corr, Position* pred, Position* optimal_pos, Position* K_gain, Position* w_avg, Quaternion* quat, Position* ZUPT) {
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

	quat->W = q_true_f32[0];
	quat->X = q_true_f32[1];
	quat->Y = q_true_f32[2];
	quat->Z = q_true_f32[3];

	ZUPT->X = (float)curr_phase; // Phase
	ZUPT->Y = (float)phase_counter;

	return w_avg_b0_mag;

}

void resetCurrentPosition(SensorData* IMU0_data, SensorData* IMU1_data) {

	int i;
	for (i = 0; i < x_curr.numRows; ++i) {
		x_curr.pData[i] = x_init_f32[i];
	}

	updatePreviousMatrices();

	clearZUPT();
	initZUPT(); // Initialize ZUPT phase detector

	initRingBuffers(IMU0_data, IMU1_data);

	initQuaternions(IMU0_data, IMU1_data);
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

void updatePreviousMatrices(void) {
	int i;
	int j;
	for (i = 0; i < x_curr.numRows; ++i) {
		x_prev.pData[i] = x_curr.pData[i];
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

	// Rotate IMU0 XL from board frame to nav frame
	arm_mat_mult_f32(&rotation_b0_n, &temp0, &temp2);	// R_b0_n*a0_b0 --> (3x3) * (3x1)

	//printf("%f %f %f \n", temp2_f32[0], temp2_f32[1], temp2_f32[2]);

	// Rotate IMU1 XL from board frame to nav frame
	arm_mat_mult_f32(&rotation_b0_n, &temp1, &temp3);	// R_b0_n*a1_b0 --> (3x3) * (3x1)

	// Subtract gravitation vector from IMU0 XL vector
	arm_mat_sub_f32(&temp2, &g_n, &temp0);

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
float returnCurrentPosition(Position* current_pos) {

	current_pos->X = (x_curr_f32[0] + x_curr_f32[3]) / 2;
	current_pos->Y = (x_curr_f32[1] + x_curr_f32[4]) / 2;
	current_pos->Z = (x_curr_f32[2] + x_curr_f32[5]) / 2;
	// Returns avg of two position values, units of meters
	return 0.;
}
