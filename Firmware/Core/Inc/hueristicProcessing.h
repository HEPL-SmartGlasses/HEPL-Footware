/*
 * hueristicProcessing.h
 *
 *  Created on: Apr 17, 2023
 *      Author: evanm
 */

#ifndef INC_HUERISTICPROCESSING_H_
#define INC_HUERISTICPROCESSING_H_

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
#define ZUPT_W 15										// Angular Rate Energy Detector Window Size (# of samples)
#define G_VARIANCE_SQ (float)0.01*0.01					// sigma_w^2
#define ZUPT_SCALE_FACTOR 1.0/(G_VARIANCE_SQ * ZUPT_W)	// 1/(sigma_w^2 * W)
#define ZUPT_THRESHOLD (float)120000						// Y' //TODO determine this better
#define PHASE_INTERVAL_THRESHOLD 7						// Double threshold

void calculatePosition(SensorData* IMU0_data, SensorData* IMU1_data, SensorData* IMU2_data, float timeDelta);

void updateDirection(float timeDelta);

void init_heuristic_processing(SensorData* IMU0_data,  SensorData* IMU1_data,  SensorData* IMU2_data);

float returnCurrentPosition(Position* returnPos);

void calculateAvgAngularRate(SensorData* IMU0_data, SensorData* IMU1_data, SensorData* IMU2_data);

void roundVal();
void calibrate(SensorData* IMU0_data, SensorData* IMU1_data, SensorData* IMU2_data, float timeDelta);
/*
 *  Updates buffer with new reading, outputs averaged reading
 */
void getNextGyroReading(SensorData* IMU0_data, SensorData* IMU1_data, SensorData* IMU2_data, float* gyroOut);

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
 *  Creates ring buffers for gyro x, y, z
 */
void initRingBuffers(SensorData* IMU0_data, SensorData* IMU1_data, SensorData* IMU2_data);

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

void resetCurrentPosition(SensorData* IMU0_data, SensorData* IMU1_data, SensorData* IMU2_data);

void initQuaternion(SensorData* IMU0_data, SensorData* IMU1_data, SensorData* IMU2_data);

#endif /* INC_HUERISTICPROCESSING_H_ */
