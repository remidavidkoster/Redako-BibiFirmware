/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "spi.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ICM-42670-P.h"
#include "NRF24L01P.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


int POLE_PAIRS = 7;
float shaft_angle;



ICM42670 imuA;
ICM42670 imu;


// NRF buffer
#define PAYLOADSIZE 12
uint8_t buffer[PAYLOADSIZE];


void configNRF(uint8_t chan) {
	// NRF24L01P init
	NRF_Init();
	setRADDR((uint8_t *)"BIBIx");
	setTADDR((uint8_t *)"BIBIx");
	payload = PAYLOADSIZE;
	channel = chan;
	NRF_Config();
}



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#include "math.h"
#include "MadgwickAHRS.h"

#define TABLE_SIZE 2048
#define TWO_PI 6.28318530718f  // 2 * π


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


void _sincos(float a, float* s, float* c){
	*s = fastSin(a);
	*c = fastCos(a);
}


float Ualpha, Ubeta; //!< Phase voltages U alpha and U beta used for inverse Park and Clarke transform
float Ua, Ub, Uc;//!< Current phase voltages Ua,Ub and Uc set to motor

#define _SQRT3_2 0.86602540378f

#define SUPPLY_VOLTAGE 11.3f

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define LIMIT(low,x,high) ((x)<(low)?(low):((x)>(high)?(high):(x)))

#define PWM_MAX ((int)8499)

#define _1_DIV_SUPPLY_VOLTAGExPWM_MAX (1/11.3f*PWM_MAX)

// Empirically determined PWM limits - strange behavior when exceeding these in up-down counting HRTIM1. Might not be relevant for normal timer but untested still.
int lowerPWMLimit = 10;
int upperPWMLimit = 8489;

int motorNumber = 0;



void setPhaseVoltage(float Uq, float angle_el) {

	float center;
	float _ca,_sa;

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

	if (0){//foc_modulation == 1){
		// discussed here: https://community.simplefoc.com/t/embedded-world-2023-stm32-cordic-co-processor/3107/165?u=candas1
		// a bit more info here: https://microchipdeveloper.com/mct5001:which-zsm-is-best
		// Midpoint Clamp
		float Umin = min(Ua, min(Ub, Uc));
		float Umax = max(Ua, max(Ub, Uc));
		center -= (Umax+Umin) * 0.5f;
	}


	Ua += center;
	Ub += center;
	Uc += center;



	// calculate duty cycle and PWM value in one
	// limited in [0,1]
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

float phaseVoltage = 0;


struct ADC {
	uint16_t valueBattery;
	uint16_t valueVRefInt;
	uint16_t vrefintCal;
	float vrefintVoltage;
	const float BAT_VOLTAGE_DIVIDER_RATIO = 2.0f;
} ADC;




struct BAT {
	float voltage;
} BAT;

struct CHG {
	uint8_t charging;
	uint8_t standby;
	uint8_t plugged;
	uint8_t enabled;
} CHG;














// Struct to hold accelerometer and gyro data, and angles
typedef struct {
	sensorXYZFloat accel;
	sensorXYZFloat gyro;
	sensorXYZFloat gyroZerod;
	sensorXYZFloat lastGyro;

	float rawAngle, angle, anglePrev, angleFull;

	int32_t turns;
} IMU_Data;

IMU_Data imu_data;  // Array to store data for two IMUs

uint32_t timestamp;
uint32_t lastTimestamp;









uint8_t failed;


typedef struct {
	float p;
	float i;
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
} PIDController;


PIDController pid = {
		.p = 0.02f,
		.i = 0.00005f,
		.d = 400.0f,
		.alpha = 0.001f,  // Set this based on how much filtering you want
		.limit = 5.65f,
		.target = 0.0f
};

float speed;

#define ABS(x) ((x) < 0 ? -(x) : (x))

float electricalAngle;

volatile float madwickAngleFull;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6) {

		pid.error = madwickAngleFull - pid.target;

		if (pid.target != pid.lastTarget){
			pid.prev_error = pid.error;
			pid.lastTarget = pid.target;
		}


		// Derivative with low-pass filter
		pid.derivative = pid.alpha * (pid.error - pid.prev_error) * pid.d + (1 - pid.alpha) * pid.derivative;

		pid.integral += pid.error * pid.i;

		pid.integral = LIMIT(-pid.limit, pid.integral, pid.limit);

		// PID output
		pid.output = pid.p * pid.error + pid.derivative + pid.integral;

		pid.output = LIMIT(-pid.limit, pid.output, pid.limit);

		pid.prev_error = pid.error;

		phaseVoltage = pid.output;

		setPhaseVoltage(phaseVoltage, electricalAngle);
	}
}




#include <string.h>  // Also works fine for embedded C/C++


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






uint8_t txData[6];
uint8_t rxData[6];
uint32_t lastRawAngle = 0;


float velocity=0.0f;
float angle_prev=0.0f; // result of last call to getSensorAngle(), used for full rotations and velocity
float angleFull=0.0f; // result of last call to getSensorAngle(), used for full rotations and velocity

float zero_electric_angle;

uint32_t angle_prev_ts=0; // timestamp of last call to getAngle, used for velocity
float vel_angle_prev=0.0f; // angle at last call to getVelocity, used for velocity
uint32_t vel_angle_prev_ts=0; // last velocity calculation timestamp
int32_t full_rotations=0; // full rotation tracking
int32_t vel_full_rotations=0; // previous full rotation value for velocity calculation

const int32_t sensor_direction = 1;



#define _2PI 6.28318530718f
#define TWO_PI 6.28318530718f  // 2 * π
#define _1_OVER_2PI 0.15915494309f  // 1 / (2 * PI)
#define _3PI_2 4.71238898038f


// normalizing radian angle to [0,2PI]
__attribute__((weak)) float _normalizeAngle(float angle){
	float norm = angle - TWO_PI * ((int)(angle * _1_OVER_2PI));
	return (norm < 0) ? (norm + TWO_PI) : norm;
}




void ENC_Update(){
	//		HAL_SPI_DeInit(&hspi2);
	//		hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;   // or HIGH
	//		hspi2.Init.CLKPhase    = SPI_PHASE_1EDGE;    // or 2EDGE
	//		HAL_SPI_Init(&hspi2);

	HAL_GPIO_WritePin(ENC_CSN_GPIO_Port, ENC_CSN_Pin, (GPIO_PinState)0);

	txData[0] = 0b1010 << 4;
	txData[1] = 0x03;

	HAL_SPI_TransmitReceive(&hspi2, txData, rxData, 6, HAL_MAX_DELAY);

	lastRawAngle = 0;

	// Extract bits from rawData1 and rawData2
	lastRawAngle |= ((uint32_t)(rxData[0 + 2]) << 13);  // Upper 8 bits (ANGLE[20:13])
	lastRawAngle |= ((uint32_t)(rxData[1 + 2]) << 5); // Middle 8 bits (ANGLE[12:5])
	lastRawAngle |= ((uint32_t)(rxData[2 + 2]) & 0b11111000) >> 3; // Lower 5 bits (ANGLE[4:0])

	HAL_GPIO_WritePin(ENC_CSN_GPIO_Port, ENC_CSN_Pin, (GPIO_PinState)1);

	float val = (2097151 - lastRawAngle) * 0.00000299605622633914f;
	//	    angle_prev_ts = TIM6->CNT;
	float d_angle = val - angle_prev;
	// if overflow happened track it as full rotation
	if(abs(d_angle) > (0.8f*_2PI) ) full_rotations += ( d_angle > 0 ) ? -1 : 1;
	angle_prev = val;

	angleFull = (float)full_rotations * _2PI + angle_prev;

	// The MT6835 SPI uses mode=3 (CPOL=1, CPHA=1) to exchange data.
	// The NRF SPI uses (CPOL=0, CPHA=0) to exchange data.
}








void BAT_Update(){
	ADC.valueVRefInt = ADC1->DR;
	ADC.valueBattery = ADC2->DR;
	BAT.voltage = (float)ADC.valueBattery / (float)ADC.valueVRefInt * ADC.vrefintVoltage * ADC.BAT_VOLTAGE_DIVIDER_RATIO;
}



void CHG_RunLogic(){

	// Charge stuff
	CHG.standby = !HAL_GPIO_ReadPin(STDBY_GPIO_Port, STDBY_Pin);
	CHG.charging = !HAL_GPIO_ReadPin(CHRG_GPIO_Port, CHRG_Pin);
	CHG.plugged = HAL_GPIO_ReadPin(VBUS_PRESENT_GPIO_Port, VBUS_PRESENT_Pin);
	CHG.enabled = BAT.voltage > 2.5f;

	// Charge enable logic
	HAL_GPIO_WritePin(CHARGE_ENABLE_GPIO_Port, CHARGE_ENABLE_Pin, (GPIO_PinState)(CHG.plugged && CHG.enabled));
}




Madgwick filter;

unsigned long microsPerReading, microsPrevious;




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


#define STABILITY_THRESHOLD 0.002f  // Threshold for accelerometer standard deviation
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


/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

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
	/* USER CODE BEGIN 2 */




	// Set VREF to 2.5V
	HAL_SYSCFG_VREFBUF_VoltageScalingConfig(SYSCFG_VREFBUF_VOLTAGE_SCALE2);



	// Start microsecond timer, overflows after 71 minutes.
	HAL_TIM_Base_Start(&htim2);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);











	HAL_GPIO_WritePin(DRIVER_ENABLE1_GPIO_Port, DRIVER_ENABLE1_Pin, (GPIO_PinState)1);
	HAL_GPIO_WritePin(BOOST_ENABLE_GPIO_Port, BOOST_ENABLE_Pin, (GPIO_PinState)1);


	ADC1->CR |= ADC_CR_ADCAL;
	while (ADC1->CR & ADC_CR_ADCAL);  // Wait until calibration is done

	ADC2->CR |= ADC_CR_ADCAL;
	while (ADC2->CR & ADC_CR_ADCAL);  // Wait until calibration is done

	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start(&hadc2);



	ADC.vrefintCal = *VREFINT_CAL_ADDR;
	ADC.vrefintVoltage = ((float)ADC.vrefintCal / 4095.0f) * 3.0f;


	// Charge power only, no button pressed
	while (!HAL_GPIO_ReadPin(BUT1_GPIO_Port, BUT1_Pin)){

		// Update global battery Voltage
		BAT_Update();

		// Display Battery on led
		BAT_VoltageToRGB(BAT.voltage);

		// Run charge management logic
		CHG_RunLogic();
	}





	// Keep itself on
	HAL_GPIO_WritePin(SELF_TURN_ON_GPIO_Port, SELF_TURN_ON_Pin, (GPIO_PinState)1);


	HAL_GPIO_WritePin(DRIVER_ENABLE1_GPIO_Port, DRIVER_ENABLE1_Pin, (GPIO_PinState)1);
	HAL_GPIO_WritePin(BOOST_ENABLE_GPIO_Port, BOOST_ENABLE_Pin, (GPIO_PinState)1);



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

	// Initialize sine lookup table
	initSinTable();




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





	configNRF(10);











	waitForStableGetGyroOffsets();


	setPhaseVoltage(5.65, _3PI_2);
	HAL_Delay(4000);
	ENC_Update();
	zero_electric_angle = _normalizeAngle((float)(POLE_PAIRS * angle_prev));
	setPhaseVoltage(0, _3PI_2);




	// Start main motor interrupt
	HAL_TIM_Base_Start_IT(&htim6);


#define SAMPLE_FREQUENCY 1000


	// Begin Madwick filter at 1000hz
	filter.begin(SAMPLE_FREQUENCY);

	// initialize variables to pace updates to correct rate
	microsPerReading = 1000000 / SAMPLE_FREQUENCY;
	microsPrevious = TIM2->CNT;

	//	while (1){
	//		// Read battery Voltage
	//		ADC.valueVRefInt = ADC1->DR;
	//		ADC.valueBattery = ADC2->DR;
	//		BAT.voltage = (float)ADC.valueBattery / (float)ADC.valueVRefInt * ADC.vrefintVoltage * ADC.BAT_VOLTAGE_DIVIDER_RATIO;
	//
	//
	//
	//		accelb = icm42670_read_accel(&imuB);
	//		angleb = atan2f(accelb.y, -accelb.x) * 180.0f / (float)M_PI;
	//
	//
	//		angleb = LIMIT(-45, angleb, 45);
	//
	//		buffer[0] = (angleb + 45.0f) * 255.0f / 90.0f;
	//
	//		NRF_Send(buffer);
	//		while (NRF_IsSending());
	//
	//
	//		BAT_VoltageToRGB(BAT.voltage);
	//		HAL_Delay(5);
	//		RGB_Set(0, 0, 0);
	//		HAL_Delay(45);
	//
	//	}




	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
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

			float roll = filter.getRoll();
			float pitch = filter.getPitch();
			float heading = filter.getYaw();

			float madwickAngle = (roll < 0) ? pitch - 90 : 90 - pitch;
			static float madwickAnglePrev;
			static int madwickTurns;




			// Assuming imu_data.angle is in [0, 360)
			float current_angleMad = madwickAngle;
			float previous_angleMad = madwickAnglePrev;

			float delta_angleMad = current_angleMad - previous_angleMad;

			// Detect wrap-around and update turn counter
			if (delta_angleMad > 180.0f) {
				madwickTurns--; // Rotated backwards across 0°
			} else if (delta_angleMad < -180.0f) {
				madwickTurns++; // Rotated forward across 360°
			}

			madwickAnglePrev = current_angleMad;

			// Compute total angle
			madwickAngleFull = current_angleMad + 360.0f * madwickTurns;





			//		if (NRF_DataReady()) {
			//			NRF_GetData(buffer);
			//
			//			pid.target = (float)buffer[0] * 180 / 255.0f - 90.0f;
			//			BAT_VoltageToRGB(BAT.voltage);
			//		}









			ENC_Update();


			electricalAngle = _normalizeAngle((float)POLE_PAIRS * angle_prev - zero_electric_angle);




			pid.target = TIM2->CNT / 3000000 & 1 ? 90 : -90;







			myData.a = roll;
			myData.b = pitch;
			myData.c = heading;
			myData.d = madwickAngleFull;
			myData.e = 0;



			sendFloats(&myData);

			//		NRF_Send(buffer);
			//		while (NRF_IsSending());
			//
			//		RGB_Set(0, 0, 0);
			//		HAL_Delay(100);
			//		BAT_VoltageToRGB(BAT.voltage);
			//		HAL_Delay(400);

			microsPrevious = microsPrevious + microsPerReading;
		}


		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
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
