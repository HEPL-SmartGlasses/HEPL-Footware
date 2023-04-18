/*
 * hueristicProcessing.c
 *
 *  Created on: Apr 17, 2023
 *      Author: evanm
 */

#include <hueristicProcessing.h>

#include "stdlib.h"
#include "stdio.h"
#include "math.h"

#define AVG_VELOCITY 1.2

#define VELOCITY_ADJUSTMENT 0.606

float personWeight = 0;

#define DIR_ADJUSTMENT -1

#define Z_TURN_THRESHOLD 13

float xOffset = 0;

float w_avg_b0_mag;

float w_avg_x_ring[RING_SIZE];
float w_avg_y_ring[RING_SIZE];
float w_avg_z_ring[RING_SIZE];

int upDownCounter = 0;


uint8_t w_oldest = 0;

float w_avg_b0_f32[3] = { // (3x1)
		0,
		0,
		0,
};

float q_f32[4] = {1,0,0,0,};

ZUPTNode* ZUPTHead; // Head to ZUPT linked list

Position pos = {0,0,0};

float heading = 0;



// ZUPT
enum PHASE curr_phase = STANCE;

// Saturating interval counter
uint8_t phase_counter = 0; // 0 --> stance side, PHASE_INTERVAL_THRESHOLD --> swing side

int8_t upDownSaturation = 50;

void calculatePosition(SensorData* IMU0_data, SensorData* IMU1_data, SensorData* IMU2_data, float timeDelta) {
	calculateAvgAngularRate(IMU0_data, IMU1_data, IMU2_data);

	float averageAcc[3];
	averageAcc[0] = (IMU0_data->XL_X + IMU1_data->XL_X + IMU2_data->XL_X) / 3;
	averageAcc[1] = (IMU0_data->XL_Y + IMU1_data->XL_Y + IMU2_data->XL_Y) / 3;
	averageAcc[2] = (IMU0_data->XL_Z + IMU1_data->XL_Z + IMU2_data->XL_Z) / 3;


	detectZUPTPhase();

	if (curr_phase == SWING) {

		if(w_avg_b0_f32[2] > Z_TURN_THRESHOLD && averageAcc[0] < xOffset - 0.07){
			if(upDownCounter > 4 ){
				updateDirection(timeDelta);
			}
			upDownCounter+= 2;
			if(upDownCounter > upDownSaturation) upDownCounter = upDownSaturation;
		}
		else if(w_avg_b0_f32[2] < -1*Z_TURN_THRESHOLD && averageAcc[0] > xOffset + 0.07){
			if(upDownCounter < -4 ){
				updateDirection(timeDelta);
			}
			upDownCounter-=2;
			if(upDownCounter < -1*upDownSaturation) upDownCounter = -1*upDownSaturation;
		}
		else if(vec_mag_f32(averageAcc) >  10.5){
			pos.X += VELOCITY_ADJUSTMENT*AVG_VELOCITY*cosf(heading*deg2rad)*timeDelta;
			pos.Y += VELOCITY_ADJUSTMENT*AVG_VELOCITY*sinf(heading*deg2rad)*timeDelta;
		}
	} else {
		upDownCounter = 0;
	}
}

void roundVal(){
	heading = roundf(heading/15) * 15;
}

extern volatile int calibrated;
int turningStarted = 0;
void calibrate(SensorData* IMU0_data, SensorData* IMU1_data, SensorData* IMU2_data, float timeDelta){
	calculateAvgAngularRate(IMU0_data, IMU1_data, IMU2_data);

	if(w_avg_b0_f32[2] > Z_TURN_THRESHOLD/4){
		updateDirection(timeDelta);
		turningStarted = 1;
	} else if (turningStarted){
		personWeight = 90.0/heading;
		calibrated = 1;
		heading = 0;
	}
}

void updateDirection(float timeDelta) {
	/*float rotation_angle_div_2 = w_avg_b0_mag * timeDelta * deg2rad;// / 2;

	float q1_3_scaling_term = (w_avg_b0_mag) ?
			(float)sin(rotation_angle_div_2) / w_avg_b0_mag : w_avg_b0_mag; // reduce number of calculations

	// Determine change in rotation as quaternion
	float delta_q_f32[4];
	delta_q_f32[0] = (float)cosf(rotation_angle_div_2);
	delta_q_f32[1] = w_avg_b0_f32[0] * q1_3_scaling_term;
	delta_q_f32[2] = w_avg_b0_f32[1] * q1_3_scaling_term;
	delta_q_f32[3] = w_avg_b0_f32[2] * q1_3_scaling_term;

	arm_quaternion_normalize_f32(delta_q_f32, delta_q_f32, 1);	// q = q / |q|a

	// Shallow copy of prev q_gyro
	float temp_q_f32[4];
	temp_q_f32[0] = q_f32[0];
	temp_q_f32[1] = q_f32[1];
	temp_q_f32[2] = q_f32[2];
	temp_q_f32[3] = q_f32[3];

	// Calculate new normalized quaternion
	arm_quaternion_product_single_f32(delta_q_f32, temp_q_f32, q_f32); // q = q x delta_q
	arm_quaternion_normalize_f32(q_f32, q_f32, 1);

	// Calculate Heading from Quaternion
	float test = q_f32[1] * q_f32[2] + q_f32[0] * q_f32[3];

	if (test > 0.499) { // North Pole Singularity
		heading = 2 * atan2f(q_f32[1], q_f32[0]);
		return;
	} else if (test < -0.499) { // South Pole Singularity
		heading = -2 * atan2f(q_f32[1], q_f32[0]);
		return;
	}

	float term1 = 2 * (q_f32[0] * q_f32[2] - q_f32[1] * q_f32[3]);
	float term2 = 1 - 2 * (q_f32[2] * q_f32[2] - q_f32[3] * q_f32[3]);
	heading = atan2f(term1, term2);*/

	if (personWeight) {
		heading += w_avg_b0_f32[2] * timeDelta * DIR_ADJUSTMENT*personWeight;
	} else {
		heading += w_avg_b0_f32[2] * timeDelta * DIR_ADJUSTMENT;
	}

	heading = (float)((int)heading % 360);

	heading = roundf(heading/5) * 5;

}

void init_heuristic_processing(SensorData* IMU0_data,  SensorData* IMU1_data,  SensorData* IMU2_data) {
	initZUPT();

	initRingBuffers(IMU0_data, IMU1_data, IMU2_data);

	initQuaternion(IMU0_data, IMU1_data, IMU2_data);

	xOffset = (IMU0_data->XL_X + IMU1_data->XL_X + IMU2_data->XL_X) / 3;


}

float returnCurrentPosition(Position* returnPos) {
	returnPos->X = pos.X;
	returnPos->Y = pos.Y;
	returnPos->Z = pos.Z;

	return heading;
}

void calculateAvgAngularRate(
		SensorData* IMU0_data,
		SensorData* IMU1_data,
		SensorData* IMU2_data) {

	getNextGyroReading(IMU0_data, IMU1_data, IMU2_data, w_avg_b0_f32);

	// Determine |w_avg_b0|
	w_avg_b0_mag = vec_mag_f32(w_avg_b0_f32);
}

void getNextGyroReading(SensorData* IMU0_data, SensorData* IMU1_data, SensorData* IMU2_data, float* gyroOut) {
	w_avg_x_ring[w_oldest] = (IMU0_data->G_X + IMU1_data->G_X + IMU2_data->G_X) / 3;
	w_avg_y_ring[w_oldest] = (IMU0_data->G_Y + IMU1_data->G_Y + IMU2_data->G_Y) / 3;
	w_avg_z_ring[w_oldest] = (IMU0_data->G_Z + IMU1_data->G_Z + IMU2_data->G_Z) / 3;

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

void resetCurrentPosition(SensorData* IMU0_data, SensorData* IMU1_data, SensorData* IMU2_data) {
	pos.X = 0;
	pos.Y = 0;

	clearZUPT();
	initZUPT(); // Initialize ZUPT phase detector

	initRingBuffers(IMU0_data, IMU1_data, IMU2_data);

	initQuaternion(IMU0_data, IMU1_data, IMU2_data);

	heading = 0;
}

void initRingBuffers(SensorData* IMU0_data, SensorData* IMU1_data, SensorData* IMU2_data) {
	float w_avg_tmp[3];
	w_avg_tmp[0] = (IMU0_data->G_X + IMU1_data->G_X + IMU2_data->G_X) / 3;
	w_avg_tmp[1] = (IMU0_data->G_Y + IMU1_data->G_Y + IMU2_data->G_Y) / 3;
	w_avg_tmp[2] = (IMU0_data->G_Z + IMU1_data->G_Z + IMU2_data->G_Z) / 3;

	int i;
	for(i = 0; i < RING_SIZE; ++i) {
		w_avg_x_ring[i] = w_avg_tmp[0];
		w_avg_y_ring[i] = w_avg_tmp[1];
		w_avg_z_ring[i] = w_avg_tmp[2];
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

void initQuaternion(SensorData* IMU0_data, SensorData* IMU1_data, SensorData* IMU2_data) {
	float avg_XL[3];
	avg_XL[0] = (IMU0_data->XL_X + IMU1_data->XL_X + IMU2_data->XL_X) / 3;
	avg_XL[1] = (IMU0_data->XL_Y + IMU1_data->XL_Y + IMU2_data->XL_Y) / 3;
	avg_XL[2] = (IMU0_data->XL_Z + IMU1_data->XL_Z + IMU2_data->XL_Z) / 3;

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
}
