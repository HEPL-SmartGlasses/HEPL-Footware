/*
 * compProcessing.h
 *
 *  Created on: Apr 17, 2023
 *      Author: evanm
 */

#ifndef INC_COMPPROCESSING_H_
#define INC_COMPPROCESSING_H_

#include "arm_math.h"
#include "IMU.h"

typedef struct {
	float X;
	float Y;
	float Z;
} Position;

typedef struct {
	float W;
	float X;
	float Y;
	float Z;
} Quaternion;

typedef struct {
	struct ZUPTNode* next;
	float w_mag_sq;
} ZUPTNode;

enum PHASE {SWING, STANCE};

#define g (float)9.80274 			 	// Local acceleration due to gravity: Ann Arbor = 9.80274
#define deg2rad (float) 0.0174532925199 // pi / 180

#define RING_SIZE 1

// ZUPT
#define ZUPT_W 20										// Angular Rate Energy Detector Window Size (# of samples)
#define G_VARIANCE_SQ (float)0.01*0.01					// sigma_w^2
#define ZUPT_SCALE_FACTOR 1.0/(G_VARIANCE_SQ * ZUPT_W)	// 1/(sigma_w^2 * W)
#define ZUPT_THRESHOLD (float)80000						// Y' //TODO determine this better
#define PHASE_INTERVAL_THRESHOLD 10					// Double threshold

void init_comp_processing(SensorData* IMU0_data, SensorData* IMU1_data);

void calculateCompCorrectedState(SensorData* IMU0_data, SensorData* IMU1_data, float timeDelta);

void euler_to_quaternion(float* XL_angle, float* quat);

void getAccelAngles(float* XL_angles);

void quatSLERP(float* q1, float* q2, float* q3, float t);

void updateGyroQuat(float timeDelta);

void updateAccelQuat(float timeDelta);

void calculateRotationMatrix(void);

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

void initQuaternions(SensorData* IMU0_data, SensorData* IMU1_data);

void calculateAvgAngularRate(IMU0_data, IMU1_data);

float returnCurrentPosition(Position* current_pos);

float returnDebugOutput(Position* corr, Position* pred, Position* optimal_pos, Position* K_gain, Position* w_avg, Quaternion* quat, Position* ZUPT);

/*
 *  Updates buffer with new reading, outputs averaged reading
 */
void getNextGyroReading(SensorData* IMU0_data, SensorData* IMU1_data, float* gyroOut);

/*
 *  Updates buffer with new reading, outputs averaged reading
 */
void getNextXLReading(SensorData* IMU0_data, SensorData* IMU1_data, float* xl0Out, float* xl1Out);

/*
 *  Creates a linked-list of size W initialized to stance phase for ZUPT
 */
void initZUPT(void);

/*
 *  Frees memory associated with ZUPT linked list
 */
void clearZUPT(void);

/*
 *  ZUPT phase detection function
 *  	Uses Angular Rate Detection based on a linked-list
 */
enum PHASE detectZUPTPhase(void);

/*
 *  Allocates memory for the ZUPT detector linked-list
 */
ZUPTNode* createZUPTNode(float w_mag);


/*
 *  Update x(k-1), P(k-1), (Q(k-1)?)
 */
void updatePreviousMatrices(void);

/*
 *  Creates ring buffers for gyro x, y, z
 */
void initRingBuffers(SensorData* IMU0_data, SensorData* IMU1_data);

/*
 *  Do dot product of two vectors
 */
float dot_f32(float* a, float* b);

/*
 *  Do cross product of two vectors
 *  REQUIRES: a, b, c must point to different vectors
 */
void cross_f32(float* a, float* b, float* c);

/*
 *  Return |vector|
 */
float vec_mag_f32(float* vec);

/*
 *  Do cross product of two inputed matrix instances (vectors)
 *  Wrapper for cross_f32
 */
void cross_product(
		arm_matrix_instance_f32* a,
		arm_matrix_instance_f32* b,
		arm_matrix_instance_f32* c);

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

void resetCurrentPosition(SensorData* IMU0_data, SensorData* IMU1_data);

#endif /* INC_COMPPROCESSING_H_ */
