/*
 * processing.c
 *
 *  Created on: Feb 27, 2023
 *      Author: evanm
 */

#include "processing.c"

void init_processing(void) {
	uint32_t numRows, numCols; // temp vars

	/*
	 * Init constant vector/matrices
	 */

	// Relative IMU position vector
	numRows = 3;
	numCols = 1;
	arm_mat_init_f32(&m_b0, numRows, numCols, m_b0_f32);

	// Gravitation vector
	numRows = 3;
	numCols = 1;
	arm_mat_init_f32(&g_n, numRows, numCols, g_n_f32);


}

void calculateAvgAngularRate(SensorData* IMU0_data, SensorData* IMU1_data) {
	w_avg_f32[0] = (IMU0_data->G_X + IMU1_data->G_X) / 2;
	w_avg_f32[1] = (IMU0_data->G_Y + IMU1_data->G_Y) / 2;
	w_avg_f32[2] = (IMU0_data->G_Z + IMU1_data->G_Z) / 2;
}

void calculateRotationMatrix(float timeDelta) {



}

