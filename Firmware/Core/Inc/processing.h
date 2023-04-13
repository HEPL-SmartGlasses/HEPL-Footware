/*
 * processing.h
 *
 *  Created on: Feb 27, 2023
 *      Author: evanm
 */

#ifndef INC_PROCESSING_H_
#define INC_PROCESSING_H_

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

#define RING_SIZE 8

// ZUPT
#define ZUPT_W 20										// Angular Rate Energy Detector Window Size (# of samples)
#define G_VARIANCE_SQ (float)0.01*0.01					// sigma_w^2
#define ZUPT_SCALE_FACTOR 1.0/(G_VARIANCE_SQ * ZUPT_W)	// 1/(sigma_w^2 * W)
#define ZUPT_THRESHOLD (float)17000						// Y' //TODO determine this better
#define PHASE_INTERVAL_THRESHOLD 10					// Double threshold

/*
 *  Define matrix variables
 */
void init_processing(SensorData* IMU0_data, SensorData* IMU1_data);

/*
 *  Resets and re-inits
 */
void resetCurrentPosition(SensorData* IMU0_data, SensorData* IMU1_data);

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

float returnDebugOutput(Position* corr, Position* pred, Position* optimal_pos, Position* K_gain, Position* w_avg, Quaternion* quat, Position* zupt);

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
		arm_matrix_instance_f32* Hi /*(Nx12)*/,
		arm_matrix_instance_f32* Ri /*(NxN)*/);

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
 *  Creates ring buffers for gyro x, y, z
 */
void initRingBuffers(void);

/*
 *  Initialize quaternion based on initial IMU acceleration
 */
void initQuaternion(SensorData* IMU0_data, SensorData* IMU1_data);

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

// Saturating interval counter
static enum PHASE curr_phase = STANCE;
static uint8_t phase_counter = 0; // 0 --> stance side, PHASE_INTERVAL_THRESHOLD --> swing side

/*
 *  ZUPT phase detection function
 *  	Uses Angular Rate Detection based on a linked-list
 */
enum PHASE detectZUPTPhase(void);

/*
 *  Allocates memory for the ZUPT detector linked-list
 */
ZUPTNode* createZUPTNode(float w_mag);

#endif /* INC_PROCESSING_H_ */
