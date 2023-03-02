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

#define g (float)9.80274 // Local acceleration due to gravity
#define deg2rad (float) 0.0174532925199 // pi / 180

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
