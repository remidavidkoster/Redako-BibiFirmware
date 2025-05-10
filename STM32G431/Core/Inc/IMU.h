/*
 * Accelerometer.h
 *
 *  Created on: May 10, 2025
 *      Author: Red
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_







/// Accelerometer stuff

typedef struct {
	sensorXYZFloat accel;
	sensorXYZFloat gyro;
	sensorXYZFloat gyroZerod;
	sensorXYZFloat lastGyro;

	float rawAngle, angle, anglePrev, angleFull;

	int32_t turns;
} IMU_Data;

IMU_Data imu_data;  // Array to store data for two IMUs

ICM42670 imu;

uint8_t failed;




/// Madwick Stuff

Madgwick filter;

#define SAMPLE_FREQUENCY 1000

typedef struct {
	volatile float angleFull;
	float anglePrev;
	float angleDelta;
	int turns;
	float currentAngle;
} MadgwickStruct;

volatile MadgwickStruct madgwick;






/// Gyro calib / init stuff

float gyro_offsets[3] = {0.0, 0.0, 0.0}; // To store the gyro offsets

// Function to calculate the standard deviation of a given array
float calculate_standard_deviation(float *data, int num_samples) {
	float sum = 0.0;
	float mean, stddev = 0.0;

	// Calculate the mean
	for (int i = 0; i < num_samples; i++) {
		sum += data[i];
	}
	mean = sum / num_samples;

	// Calculate the standard deviation
	for (int i = 0; i < num_samples; i++) {
		stddev += pow(data[i] - mean, 2);
	}
	return sqrt(stddev / num_samples);
}


#define STABILITY_THRESHOLD 0.003f  // Threshold for accelerometer standard deviation
#define NUM_SAMPLES 100          // Number of samples to collect for both accelerometer and gyro
#define GRAVITY 9.81f             // Gravity constant in m/s^2

float accel_x_stddev;
float accel_y_stddev;
float accel_z_stddev;


void waitForStableGetGyroOffsets(){
	while (1) {
		// Arrays to store accelerometer and gyro samples
		float accel_x_samples[NUM_SAMPLES] = {0};
		float accel_y_samples[NUM_SAMPLES] = {0};
		float accel_z_samples[NUM_SAMPLES] = {0};
		float gyro_x_samples[NUM_SAMPLES] = {0};
		float gyro_y_samples[NUM_SAMPLES] = {0};
		float gyro_z_samples[NUM_SAMPLES] = {0};

		// Collect NUM_SAMPLES samples of accelerometer and gyro data
		for (int i = 0; i < NUM_SAMPLES; i++) {
			imu_data.accel = icm42670_read_accel(&imu); // Replace with actual accel read function
			imu_data.gyro = icm42670_read_gyro(&imu);   // Replace with actual gyro read function

			// Store the accelerometer and gyro samples
			accel_x_samples[i] = imu_data.accel.x;
			accel_y_samples[i] = imu_data.accel.y;
			accel_z_samples[i] = imu_data.accel.z;
			gyro_x_samples[i] = imu_data.gyro.x;
			gyro_y_samples[i] = imu_data.gyro.y;
			gyro_z_samples[i] = imu_data.gyro.z;

			// Optional: Add a small delay between samples if necessary
			HAL_Delay(10);

			// Check for button shutdown
			if (!HAL_GPIO_ReadPin(BUT2_GPIO_Port, BUT2_Pin)) HAL_GPIO_WritePin(SELF_TURN_ON_GPIO_Port, SELF_TURN_ON_Pin, (GPIO_PinState)0);

			// And turn off after 2 minutes of not having become stable
			if (TIM2->CNT > 120000000) HAL_GPIO_WritePin(SELF_TURN_ON_GPIO_Port, SELF_TURN_ON_Pin, (GPIO_PinState)0);
		}

		// Calculate the standard deviation for each axis of the accelerometer
		accel_x_stddev = calculate_standard_deviation(accel_x_samples, NUM_SAMPLES);
		accel_y_stddev = calculate_standard_deviation(accel_y_samples, NUM_SAMPLES);
		accel_z_stddev = calculate_standard_deviation(accel_z_samples, NUM_SAMPLES);

		// If the standard deviation of all axes is below the threshold, the accelerometer is stable
		if (accel_x_stddev < STABILITY_THRESHOLD && accel_y_stddev < STABILITY_THRESHOLD && accel_z_stddev < STABILITY_THRESHOLD) {
			//printf("Accelerometer is stable. Proceeding to gyro offset calculation...\n");

			// Calculate the average of the gyro readings
			float sum_gyro[3] = {0.0f, 0.0f, 0.0f};
			for (int i = 0; i < NUM_SAMPLES; i++) {
				sum_gyro[0] += gyro_x_samples[i];
				sum_gyro[1] += gyro_y_samples[i];
				sum_gyro[2] += gyro_z_samples[i];
			}

			gyro_offsets[0] = sum_gyro[0] / NUM_SAMPLES;
			gyro_offsets[1] = sum_gyro[1] / NUM_SAMPLES;
			gyro_offsets[2] = sum_gyro[2] / NUM_SAMPLES;

			//printf("Gyro Offsets: X: %f, Y: %f, Z: %f\n", gyro_offsets[0], gyro_offsets[1], gyro_offsets[2]);
			break;  // Exit the loop since we have successfully calculated the gyro offsets
		} else {
			//printf("Accelerometer is not stable. Retrying...\n");
		}

		// Optional: Add a small delay before trying again if needed
		// usleep(10000); // Sleep for 10ms (if required)
	}
}




//	const uint8_t GYRO_UI_FILT_BW_180HZ = 0b001;
//
//	icm42670_write(&imu, ICM42670_REG_GYRO_CONFIG1, &GYRO_UI_FILT_BW_180HZ, 1);

/*
000: Low pass filter bypassed
001: 180 Hz
010: 121 Hz
011: 73 Hz
100: 53 Hz
101: 34 Hz
110: 25 Hz
111: 16 Hz
 */

//	const uint8_t ACCEL_UI_FILT_BW_16HZ = 0b111;
//	const uint8_t ACCEL_UI_FILT_BW_34HZ = 0b101;
//
//	icm42670_write(&imu, ICM42670_REG_ACCEL_CONFIG1, &ACCEL_UI_FILT_BW_34HZ, 1);



#endif /* INC_IMU_H_ */
