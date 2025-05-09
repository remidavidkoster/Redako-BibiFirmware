#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "spi.h"

#include "ICM-42670-P.h"
#include "NRF24L01P.h"
#include "math.h"
#include "MadgwickAHRS.h"
#include <string.h>


// Identification number of this bibi
#define BIBI_NUMBER 4



void SystemClock_Config(void);

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define LIMIT(low,x,high) ((x)<(low)?(low):((x)>(high)?(high):(x)))
#define ABS(x) ((x) < 0 ? -(x) : (x))

#define POLE_PAIRS 7
#define _1_DIV_POLE_PAIRS 0.14285714285714285714f

#define DIABOLO_CIRCUMFERENCE 0.408407044966673121f



/// Radio Stuff

// NRF buffer
#define PAYLOADSIZE 4

typedef struct __attribute__((packed)) {
	uint8_t what;
	uint8_t who;
	uint8_t maxSpeed;          // per 0.01
	uint8_t accelerationRatio; // per 0.01

} ControlData;

uint8_t buffer[PAYLOADSIZE];
ControlData txData;
ControlData rxData;

#define CONTROL_BIBI 10
#define START_CUE 14

#define MODE_TEST 150
#define MODE_FIRE 200

#define UNDEFINED 0;
#define BIBI_CONTROLLED 1
#define CUE_CONTROLLED 2
#define CONTROLLING_BIBI 3

uint8_t BIBI_Mode;


// Debug send command
uint8_t send;

void configNRF(uint8_t chan) {
	// NRF24L01P init
	NRF_Init();
	setRADDR((uint8_t *)"TCMfx");
	setTADDR((uint8_t *)"TCMfx");
	payload = PAYLOADSIZE;
	channel = chan;
	NRF_Config();
}

uint32_t NRF_ReceiveTimestamp;
uint32_t NRF_ReceiveInterval;



/// Sine math stuff

#define TABLE_SIZE 2048
#define TWO_PI 6.28318530718f  // 2 * π
#define _SQRT3_2 0.86602540378f
#define _1_OVER_2PI 0.15915494309f  // 1 / (2 * PI)
#define _3PI_2 4.71238898038f
#define RAD2DEG 57.29577951308232087680f
#define DEG2RAD 0.01745329251994329577f
#define PI 3.141592653589793f

#define METERS2RAD (TWO_PI / DIABOLO_CIRCUMFERENCE)
#define RAD2METERS (DIABOLO_CIRCUMFERENCE / TWO_PI)



float sinTable[TABLE_SIZE];

void initSinTable() {
	for (int i = 0; i < TABLE_SIZE; i++) {
		sinTable[i] = sinf((TWO_PI * i) / TABLE_SIZE);
	}
}

// Fast sine lookup. Divide gets precomputed here
inline float fastSin(float x) {
	int index = (int)((x * (TABLE_SIZE / TWO_PI))) & (TABLE_SIZE - 1);
	return sinTable[index];
}

// Fast cosine using phase shift
inline float fastCos(float x) {
	return fastSin(x + (TWO_PI / 4));  // cos(x) = sin(x + π/2)
}

// Single SIN/COS function for FOC
inline void _sincos(float a, float* s, float* c){
	*s = fastSin(a);
	*c = fastCos(a);
}

// normalizing radian angle to [0,2PI]
inline float _normalizeAngle(float angle){
	float norm = angle - TWO_PI * ((int)(angle * _1_OVER_2PI));
	return (norm < 0) ? (norm + TWO_PI) : norm;
}





/// PWM / Motor Stuff

#define SUPPLY_VOLTAGE 11.3f

#define PWM_MAX ((int)8499)

#define _1_DIV_SUPPLY_VOLTAGExPWM_MAX (1/SUPPLY_VOLTAGE*PWM_MAX)

// Arbitrary PWM limits for now
int lowerPWMLimit = 10;
int upperPWMLimit = 8489;

int motorNumber = 0;

// Default open loop phase voltage
float phaseVoltage = 0;

// Current phase voltages Ua,Ub and Uc set to motor [V]
float Ua, Ub, Uc;

// Main FOC Function
void setPhaseVoltage(float Uq, float angle_el) {
	float Ualpha, Ubeta, center, _ca,_sa;

	// Sinusoidal PWM modulation
	// Inverse Park + Clarke transformation
	_sincos(angle_el, &_sa, &_ca);

	// Inverse park transform
	Ualpha =  -_sa * Uq;  // -sin(angle) * Uq;
	Ubeta =  _ca * Uq;    //  cos(angle) * Uq;

	// Clarke transform
	Ua = Ualpha;
	Ub = -0.5f * Ualpha + _SQRT3_2 * Ubeta;
	Uc = -0.5f * Ualpha - _SQRT3_2 * Ubeta;

	center = SUPPLY_VOLTAGE * 0.5f;

	if (1){//foc_modulation == 1){
		// Midpoint Clamp
		float Umin = min(Ua, min(Ub, Uc));
		float Umax = max(Ua, max(Ub, Uc));
		center -= (Umax+Umin) * 0.5f;
	}

	// Add center voltages [V]
	Ua += center;
	Ub += center;
	Uc += center;


	// calculate duty cycle and PWM value in one go. limited in [0,1]
	uint16_t dc_a = Ua * _1_DIV_SUPPLY_VOLTAGExPWM_MAX;
	uint16_t dc_b = Ub * _1_DIV_SUPPLY_VOLTAGExPWM_MAX;
	uint16_t dc_c = Uc * _1_DIV_SUPPLY_VOLTAGExPWM_MAX;

	// set the voltages in driver
	uint16_t A = LIMIT(lowerPWMLimit, dc_a, upperPWMLimit);
	uint16_t B = LIMIT(lowerPWMLimit, dc_b, upperPWMLimit);
	uint16_t C = LIMIT(lowerPWMLimit, dc_c, upperPWMLimit);

	if (motorNumber == 0 || motorNumber == 1){
		TIM1->CCR1 = A;
		TIM1->CCR2 = B;
		TIM1->CCR3 = C;
	}
	if (motorNumber == 0 || motorNumber == 2){
		TIM8->CCR1 = A;
		TIM8->CCR2 = B;
		TIM8->CCR3 = C;
	}
}





/// RGB LED Stuff

#define RGB_PWM_MAX 8499

void RGB_Set(float r, float g, float b){
	TIM3->CCR3 = RGB_PWM_MAX - r * RGB_PWM_MAX;
	TIM3->CCR1 = RGB_PWM_MAX - g * RGB_PWM_MAX;
	TIM3->CCR2 = RGB_PWM_MAX - b * RGB_PWM_MAX;
}

// Voltage thresholds for color transitions
#define VBAT_MIN_VOLTAGE   3.2f  // Red
#define VBAT_MID_VOLTAGE   3.7f  // Green
#define VBAT_MAX_VOLTAGE   4.2f  // Blue

void BAT_VoltageToRGB(float voltage) {
	float r = 0.0f, g = 0.0f, b = 0.0f;

	if (voltage == 0) {
		RGB_Set(r, g, b);
		return;
	}

	// Clamp voltage between min and max
	if (voltage < VBAT_MIN_VOLTAGE) voltage = VBAT_MIN_VOLTAGE;
	if (voltage > VBAT_MAX_VOLTAGE) voltage = VBAT_MAX_VOLTAGE;

	if (voltage <= VBAT_MID_VOLTAGE) {
		// Interpolate from Red (1,0,0) to Green (0,1,0)
		float t = (voltage - VBAT_MIN_VOLTAGE) / (VBAT_MID_VOLTAGE - VBAT_MIN_VOLTAGE);
		r = 1.0f - t;
		g = t;
		b = 0.0f;
	} else {
		// Interpolate from Green (0,1,0) to Blue (0,0,1)
		float t = (voltage - VBAT_MID_VOLTAGE) / (VBAT_MAX_VOLTAGE - VBAT_MID_VOLTAGE);
		r = 0.0f;
		g = 1.0f - t;
		b = t;
	}

	RGB_Set(r, g, b);
}





/// ADC Stuff

struct ADC {
	uint16_t valueBattery;
	uint16_t valueVRefInt;
	uint16_t vrefintCal;
	float vrefintVoltage;
	const float BAT_VOLTAGE_DIVIDER_RATIO = 2.0f;
} ADC;





/// Battery Stuff

struct BAT {
	float voltage;
} BAT;

struct CHG {
	uint8_t charging;
	uint8_t standby;
	uint8_t plugged;
	uint8_t enabled;
} CHG;

void BAT_Update(){
	ADC.valueVRefInt = ADC1->DR;
	ADC.valueBattery = ADC2->DR;
	BAT.voltage = (float)ADC.valueBattery / (float)ADC.valueVRefInt * ADC.vrefintVoltage * ADC.BAT_VOLTAGE_DIVIDER_RATIO;
}





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





/// PID Stuff

typedef struct {
	float p;
	float d;

	float error;
	float prev_error;
	float derivative;
	float integral;
	float output;

	float dt;
	float alpha;  // Low-pass filter factor for the derivative term

	float limit;

	float target;
	float lastTarget;

	float on;
} PIDController;


volatile PIDController pid = {
		.p = 0.000003f,
		.d = 0.01f,
		.alpha = 0.001f,  // Set this based on how much filtering you want
		.limit = 2.0f,
		.target = 0.0f,
};

volatile float speed;
float electricalAngle;
float motorAngleFull;
float diaboloAngleFull;
float diaboloPosition;
float lastDiaboloPosition;
volatile float diaboloSpeed;
float lastDiaboloSpeed;
float diaboloAcceleration;

float SPEED_ALPHA = 0.01f;
float ACCEL_ALPHA = 0.005f;


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





/// Float send debug stuff

#pragma pack(push, 1)
typedef struct {
	float a;
	float b;
	float c;
	float d;
	float e;
	float f;
} FloatStruct;
#pragma pack(pop)

FloatStruct myData;

float reverseFloatBytes(float input) {
	uint8_t *bytes = (uint8_t *)&input;
	uint8_t reversed[4];

	reversed[0] = bytes[3];
	reversed[1] = bytes[2];
	reversed[2] = bytes[1];
	reversed[3] = bytes[0];

	float output;
	memcpy(&output, reversed, sizeof(float));
	return output;
}

void sendFloats(FloatStruct *data) {
	FloatStruct reversedData;

	reversedData.a = reverseFloatBytes(data->a);
	reversedData.b = reverseFloatBytes(data->b);
	reversedData.c = reverseFloatBytes(data->c);
	reversedData.d = reverseFloatBytes(data->d);
	reversedData.e = reverseFloatBytes(data->e);
	reversedData.f = reverseFloatBytes(data->f);

	HAL_UART_Transmit(&huart2, (uint8_t *)&reversedData, sizeof(FloatStruct), 1000);
}





/// Charge stuff

void CHG_RunLogic(){

	// Charge stuff
	CHG.standby = !HAL_GPIO_ReadPin(STDBY_GPIO_Port, STDBY_Pin);
	CHG.charging = !HAL_GPIO_ReadPin(CHRG_GPIO_Port, CHRG_Pin);
	CHG.plugged = HAL_GPIO_ReadPin(VBUS_PRESENT_GPIO_Port, VBUS_PRESENT_Pin);
	CHG.enabled = BAT.voltage > 2.5f;

	// Charge enable logic
	HAL_GPIO_WritePin(CHARGE_ENABLE_GPIO_Port, CHARGE_ENABLE_Pin, (GPIO_PinState)(CHG.plugged && CHG.enabled));
}





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


#define STABILITY_THRESHOLD 0.0025f  // Threshold for accelerometer standard deviation
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











/// Main 10kHz pid loop

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6) {

		pid.error = madgwick.angleFull - pid.target;

		if (pid.target != pid.lastTarget){
			pid.prev_error = pid.error;
			pid.lastTarget = pid.target;
		}

		// Derivative with low-pass filter
		pid.derivative = pid.alpha * (pid.error - pid.prev_error) * pid.d + (1 - pid.alpha) * pid.derivative;

		// PID output
		pid.output = pid.p * pid.error + pid.derivative;// + pid.integral;

		pid.output = LIMIT(-pid.limit, pid.output, pid.limit);

		pid.prev_error = pid.error;

		//phaseVoltage = pid.output;

		if (pid.on){
			speed += pid.output;

			speed = LIMIT(-pid.limit, speed, pid.limit);

			// Speed should be in meters per second

			electricalAngle += speed * METERS2RAD * POLE_PAIRS / 10000.0f;
		}
		else {
			speed = 0;
		}

		motorAngleFull = electricalAngle / POLE_PAIRS * RAD2DEG;

		setPhaseVoltage(phaseVoltage, electricalAngle);
	}
}




// Get position at time t during cosine motion profile
// Inputs:
//   t        - time since start [s]
//   duration - total movement time [s]
//   distance - total distance to travel [m]
// Output:
//   returns position traveled [m] at time t
float get_position(uint32_t t, uint32_t duration, float distance) {
	if (t <= 0) return 0.0f;
	if (t >= duration) return distance;

	float omega = TWO_PI / duration;
	float pos = (distance / duration) * (t - (sin(omega * t) / omega));
	return pos;
}


struct Movement {
	uint32_t start;
	uint32_t running;
	uint32_t startTimestamp;
	float accDistance;
	float accAngle;
	float coastDistance;
	float decAngle;
	float startOffset;
	int direction;
	int step;
};

struct Movement movement = {
		.start = 0,
		.running = 0,
		.startTimestamp = 0,
		.startOffset = 0.0f,
		.direction = 1
};

enum {
	LEFT = -1,
	RIGHT = 1,
};


enum {
	ACCELERATING,
	COASTING,
	DECELERATING,
	STOPPING
};




/// Timing stuff

unsigned long microsPerReading, microsPrevious, microsUsed;









/// Quintic Movement Stuff

typedef struct {
	float rampRatio; // Determines max acceleration used
	float maxSpeed;
	float totalDistance;

	float rampTime;
	float cruiseTime;
	float totalTime;
	float rampDistance;
	float cruiseDistance;
} PositionProfile;

PositionProfile p = {
		.rampRatio = 0.1f,
		.maxSpeed = 0.1f,
		.totalDistance = 0.2f,
		.rampTime = 0.0f,
		.cruiseTime = 0.0f,
		.totalTime = 0.0f,
		.rampDistance = 0.0f,
		.cruiseDistance = 0.0f
};



void distanceSpeedRampRatioToProfileTimes(PositionProfile &p){
	// Calculate ramp up / ramp down time
	p.rampTime = p.maxSpeed / p.rampRatio;

	// Calculate total distance spent ramping up and down
	p.rampDistance = p.rampTime * p.maxSpeed;

	// Check if we can accelerate fast enough to reach max Speed
	if (p.rampDistance <= p.totalDistance){

		// We can! Calculate distance to spend cruising
		p.cruiseDistance = p.totalDistance - p.rampDistance;

		// Calculate time spent cruising
		p.cruiseTime = p.cruiseDistance / p.maxSpeed;
	}

	// If we can't reach max speed
	else {
		// We're fucked. Gotta recalculate the trajectory

		// The ramp ratio determines acceleration. That is fixed. Max speed becomes irrelevant if we can't reach that.
		// With a ramp ratio of 1 we can reach 1 speed in 1 time, and we'd move 0.5 distance. In total we'd move 1 distance in 2 time.
		// So for a ramp ratio of 1, the ramp time is 1 if we need to reach 1 distance.
		// For 4 distance, with a ramp ratio of 1, we'd reach 2 speed in 2 time. Moving 2 distance (0.5 * 2 * 2). Ramp time of 2.
		// With a ramp ratio of 2, we'd reach 2 speed in 1 time. Moving 1 distance in each half. Total distance of 2. Ramp time of 1.

		p.rampTime = sqrtf(p.totalDistance / p.rampRatio);
		p.maxSpeed = sqrtf(p.totalDistance * p.rampRatio);

		p.cruiseDistance = 0;
		p.cruiseTime = 0;
	}

	p.totalTime = 2 * p.rampTime + p.cruiseTime;
}





// This function gets called by the firmware reading the messages from the wireless module when it should give a cue.
void CUE_Start(uint32_t cue){

	// If the first cue should be started, and the Bibi receiving it is Bibi number 1.
	if (cue == 1 && BIBI_NUMBER == 1){

		// Move direction is to the left (seen from audience)
		movement.direction = LEFT;

		// Accelerate for 0.4 meters
		movement.accDistance = 0.4;

		// The counter weight angle when accelerating [0-90]
		movement.accAngle = 45;

		// The distance to coast for (pretty much keeps the same speed here)
		movement.coastDistance = 1.2;

		// The counter weight angle to decelerate at [0-90] (no distance is set here, she decelerates until she stops)
		movement.decAngle = 45;

		// Set the start flag high, which signals another part of the code to start the movement
		movement.start = 1;
	}

	if (cue == 2 && BIBI_NUMBER == 2){
		movement.direction = RIGHT;
		movement.accDistance = 0.4;
		movement.accAngle = 45;
		movement.coastDistance = 0.6;
		movement.decAngle = 45;
		movement.start = 1;
	}
	if (cue == 2 && BIBI_NUMBER == 3){
		HAL_Delay(1000);
		movement.direction = LEFT;
		movement.accDistance = 0.4;
		movement.accAngle = 45;
		movement.coastDistance = 0.6;
		movement.decAngle = 45;
		movement.start = 1;
	}
	if (cue == 2 && BIBI_NUMBER == 4){
		HAL_Delay(8000);
		movement.direction = RIGHT;
		movement.accDistance = 0.4;
		movement.accAngle = 45;
		movement.coastDistance = 0.6;
		movement.decAngle = 45;
		movement.start = 1;
	}
}










// Main loop
int main(void) {

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C2_Init();
	MX_TIM1_Init();
	MX_TIM6_Init();
	MX_ADC1_Init();
	MX_SPI2_Init();
	MX_ADC2_Init();
	MX_TIM3_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();



	// Start microsecond timer, overflows after 71 minutes.
	HAL_TIM_Base_Start(&htim2);



	/// Do ADC Stuff

	// Set VREF to 2.5V
	HAL_SYSCFG_VREFBUF_VoltageScalingConfig(SYSCFG_VREFBUF_VOLTAGE_SCALE2);

	ADC1->CR |= ADC_CR_ADCAL;
	while (ADC1->CR & ADC_CR_ADCAL);  // Wait until calibration is done

	ADC2->CR |= ADC_CR_ADCAL;
	while (ADC2->CR & ADC_CR_ADCAL);  // Wait until calibration is done

	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start(&hadc2);

	ADC.vrefintCal = *VREFINT_CAL_ADDR;
	ADC.vrefintVoltage = ((float)ADC.vrefintCal / 4095.0f) * 3.0f;


	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

	// Charge power only, no button pressed
	while (!HAL_GPIO_ReadPin(BUT1_GPIO_Port, BUT1_Pin)){

		// Update global battery Voltage
		BAT_Update();

		// Display Battery on led
		BAT_VoltageToRGB(BAT.voltage);

		// Run charge management logic
		CHG_RunLogic();
	}


	// If second button is pressed during startup, this will be a controller
	if (!HAL_GPIO_ReadPin(BUT2_GPIO_Port, BUT2_Pin)){
		BIBI_Mode = CONTROLLING_BIBI;
	}


	// Keep itself on
	HAL_GPIO_WritePin(SELF_TURN_ON_GPIO_Port, SELF_TURN_ON_Pin, (GPIO_PinState)1);





	//ICM42670 Init
	if(icm42670_init(&imu, ICM42670_DEFAULT_ADDRESS, &hi2c2) != HAL_OK){
		failed = 1;
	}

	// Setup rate & scale
	icm42670_mclk_on(&imu);
	icm42670_start_accel(&imu, ICM42670_ACCEL_FS_2G, ICM42670_ODR_1600_HZ);
	icm42670_start_gyro(&imu, ICM42670_GYRO_FS_2000_DPS, ICM42670_ODR_1600_HZ);

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




	configNRF(101);




	if (BIBI_Mode == CONTROLLING_BIBI){

		// Wait for button release
		while (!HAL_GPIO_ReadPin(BUT2_GPIO_Port, BUT2_Pin));

		while (1){

			// Send messages every 10ms
			if (TIM2->CNT - microsPrevious >= 10000) {

				// Update global battery Voltage
				BAT_Update();

				// Display Battery on led
				BAT_VoltageToRGB(BAT.voltage * ((TIM2->CNT / 500000) & 1));

				// Run charge management logic
				CHG_RunLogic();

				// Low battery shut down
				if (TIM2->CNT > 1000000){
					if (BAT.voltage < 3.0f) HAL_GPIO_WritePin(SELF_TURN_ON_GPIO_Port, SELF_TURN_ON_Pin, (GPIO_PinState)0);
				}

				// Button shut down
				if (!HAL_GPIO_ReadPin(BUT2_GPIO_Port, BUT2_Pin)) HAL_GPIO_WritePin(SELF_TURN_ON_GPIO_Port, SELF_TURN_ON_Pin, (GPIO_PinState)0);

				// Read accelerometer
				imu_data.accel = icm42670_read_accel(&imu);

				// Compute gravity angle
				imu_data.angle = atan2f(imu_data.accel.y, -imu_data.accel.x) * 180.0f / (float)M_PI;

				// Set command
				buffer[0] = CONTROL_BIBI;

				// Set angle
				buffer[1] = (LIMIT(-45.0f, imu_data.angle, 45.0f) + 45.0f) / 90.0f * 255.0f;

				// Send data
				NRF_Send(buffer);
				while (NRF_IsSending());

				// Increase timestamp
				microsPrevious = microsPrevious + microsPerReading;
			}
		}
	}







	// Enable motor stuff
	HAL_GPIO_WritePin(DRIVER_ENABLE1_GPIO_Port, DRIVER_ENABLE1_Pin, (GPIO_PinState)1);
	HAL_GPIO_WritePin(BOOST_ENABLE_GPIO_Port, BOOST_ENABLE_Pin, (GPIO_PinState)1);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);



	// Initialize sine lookup table
	initSinTable();











	waitForStableGetGyroOffsets();


	//	setPhaseVoltage(5.65, _3PI_2);
	//	HAL_Delay(4000);
	//	ENC_Update();
	//	zero_electric_angle = _normalizeAngle((float)(POLE_PAIRS * angle_prev));
	//	setPhaseVoltage(0, _3PI_2);









	// Begin Madwick filter at 1000hz
	filter.begin(SAMPLE_FREQUENCY);

	// initialize variables to pace updates to correct rate
	microsPerReading = 1000000 / SAMPLE_FREQUENCY;
	microsPrevious = TIM2->CNT;

	// Start main motor interrupt
	HAL_TIM_Base_Start_IT(&htim6);

	while (1)  {

		if (TIM2->CNT - microsPrevious >= microsPerReading) {

			// Update global battery Voltage
			BAT_Update();

			// Display Battery on led
			BAT_VoltageToRGB(BAT.voltage);

			// Run charge management logic
			CHG_RunLogic();

			// Low battery shut down
			if (TIM2->CNT > 1000000){
				if (BAT.voltage < 3.0f) HAL_GPIO_WritePin(SELF_TURN_ON_GPIO_Port, SELF_TURN_ON_Pin, (GPIO_PinState)0);
			}

			// Button shut down
			if (!HAL_GPIO_ReadPin(BUT2_GPIO_Port, BUT2_Pin)) HAL_GPIO_WritePin(SELF_TURN_ON_GPIO_Port, SELF_TURN_ON_Pin, (GPIO_PinState)0);




			// Read accelerometer and gyro data for IMU B (6.7mm offset above point of rotation. X+ is down. Y+ to the is right.)
			imu_data.accel = icm42670_read_accel(&imu);
			imu_data.gyro = icm42670_read_gyro(&imu);

			imu_data.gyroZerod.x = imu_data.gyro.x - gyro_offsets[0];
			imu_data.gyroZerod.y = imu_data.gyro.y - gyro_offsets[1];
			imu_data.gyroZerod.z = imu_data.gyro.z - gyro_offsets[2];

			// update the filter, which computes orientation
			filter.updateIMU(imu_data.gyroZerod.x, imu_data.gyroZerod.y, imu_data.gyroZerod.z, imu_data.accel.x, imu_data.accel.y, imu_data.accel.z);


			// Assuming imu_data.angle is in [0, 360)
			madgwick.currentAngle = (filter.getRoll() < 0) ? filter.getPitch() - 90.0f : 90.0f - filter.getPitch();
			madgwick.angleDelta = madgwick.currentAngle - madgwick.anglePrev;
			madgwick.anglePrev = madgwick.currentAngle;

			// Detect wrap-around and update turn counter
			if      (madgwick.angleDelta >  180.0f) madgwick.turns--; // Rotated backwards across 0°
			else if (madgwick.angleDelta < -180.0f) madgwick.turns++; // Rotated forward across 360°

			// Compute total angle
			madgwick.angleFull = madgwick.currentAngle + 360.0f * madgwick.turns;



			motorAngleFull = electricalAngle / POLE_PAIRS * RAD2DEG;

			// Compute the angle the diabolo has made from its startup position
			diaboloAngleFull = motorAngleFull + madgwick.angleFull;

			diaboloPosition = diaboloAngleFull / 360.0f * DIABOLO_CIRCUMFERENCE;

			diaboloSpeed = SPEED_ALPHA * (diaboloPosition - lastDiaboloPosition) * 1000.0f + (1.0f - SPEED_ALPHA) * diaboloSpeed;

			diaboloAcceleration = ACCEL_ALPHA * (diaboloSpeed - lastDiaboloSpeed) * 1000.0f + (1.0f - ACCEL_ALPHA) * diaboloAcceleration;

			lastDiaboloPosition = diaboloPosition;
			lastDiaboloSpeed = diaboloSpeed;





			if (movement.start){
				movement.start = 0;
				movement.running = 1;
				movement.startTimestamp = TIM2->CNT;
				movement.startOffset = diaboloPosition;
				movement.step = ACCELERATING;

				pid.on = 1;
				pid.target = movement.accAngle * (0 - movement.direction);
				phaseVoltage = 5;
			}


			if (movement.running) {
				if (movement.direction == RIGHT){
					if ((movement.step == ACCELERATING) && ((diaboloPosition - movement.startOffset) > movement.accDistance)){
						movement.step = COASTING;
						pid.target = 0;
					}
					if ((movement.step == COASTING) && ((diaboloPosition - movement.startOffset) > (movement.accDistance + movement.coastDistance))){
						movement.step = DECELERATING;
						pid.target = movement.decAngle * movement.direction;
					}
					if ((movement.step == DECELERATING) && diaboloSpeed < 0){
						movement.step = STOPPING;
						pid.target = 0;
						movement.running = 0;
						phaseVoltage = 2;
						pid.on = 0;
					}
				}

				if (movement.direction == LEFT){
					if ((movement.step == ACCELERATING) && ((movement.startOffset - diaboloPosition) > movement.accDistance)){
						movement.step = COASTING;
						pid.target = 0;
					}
					if ((movement.step == COASTING) && ((movement.startOffset - diaboloPosition) > (movement.accDistance + movement.coastDistance))){
						movement.step = DECELERATING;
						pid.target = movement.decAngle * movement.direction;
					}
					if ((movement.step == DECELERATING) && diaboloSpeed > 0){
						movement.step = STOPPING;
						pid.target = 0;
						movement.running = 0;
						phaseVoltage = 2;
						pid.on = 0;
					}
				}
			}


			if (TIM2->CNT - NRF_ReceiveTimestamp > 20000){
				//	    		pid.target = 0;
			}
			if (TIM2->CNT - NRF_ReceiveTimestamp > 2000000){
				//	    		phaseVoltage = 0;
				//	    		speed = 0;

				// Turn bibi controlled mode off if no message for 2 seconds.
				if (BIBI_Mode == BIBI_CONTROLLED){
					BIBI_Mode = UNDEFINED;
				}
			}


			if (NRF_DataReady()) {
				NRF_GetData(buffer);
				NRF_ReceiveInterval = TIM2->CNT - NRF_ReceiveTimestamp;
				NRF_ReceiveTimestamp = TIM2->CNT;

				if (buffer[0] == START_CUE && (buffer[2] == MODE_TEST || buffer[2] == MODE_FIRE) && BIBI_Mode != BIBI_CONTROLLED){
					if (buffer[3] == 1){
						CUE_Start(1);
					}
					else if (buffer[3] == 2){
						CUE_Start(2);
					}
					else if (buffer[3] == 4){
						CUE_Start(3);
					}
					else if (buffer[3] == 8){
						CUE_Start(4);
					}
				}

#define CONTROL_BIBI_MAX_ANGLE 75.0f
#define CONTROL_BIBI_MAX_POWER_ANGLE 90.0f

				if (buffer[0] == CONTROL_BIBI && BIBI_NUMBER == 1){
					pid.target = buffer[1] / 255.0f * CONTROL_BIBI_MAX_ANGLE * 2 - CONTROL_BIBI_MAX_ANGLE;
					phaseVoltage = 5.0f + LIMIT(0, ABS(pid.target) / CONTROL_BIBI_MAX_POWER_ANGLE, 1.5f);
					pid.on = 1;

					BIBI_Mode = BIBI_CONTROLLED;
				}
			}



			// Debug send command
			if (send == 1) {
				send = 0;

				NRF_Send(buffer);
				while (NRF_IsSending());
			}


			// pid.target = TIM2->CNT / 4000000 & 1 ? 45 : -45;


			myData.a = diaboloPosition;
			myData.b = diaboloSpeed;
			myData.c = diaboloAcceleration;
			myData.d = 0;
			myData.e = 0;

			sendFloats(&myData);

			microsUsed = TIM2->CNT - microsPrevious - microsPerReading;

			microsPrevious = microsPrevious + microsPerReading;
		}
	}
}


void SystemClock_Config(void){
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
}

void Error_Handler(void){
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
