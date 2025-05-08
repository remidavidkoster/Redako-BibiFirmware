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
#define MIN_PAYLOAD 4
#define MAX_PAYLOAD 12
#define MAX_CHANNEL 125

uint8_t buffer[MAX_PAYLOAD];

void configNRF(uint8_t chan, uint8_t psize) {
    NRF_Init();
    setRADDR((uint8_t *)"TCMfx");   // RX address
    setTADDR((uint8_t *)"TCMfx");   // Not needed, but initialized
    payload = psize;
    channel = chan;
    NRF_Config();
}





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
float phaseVoltage = 2;

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
		.p = 2.99999989e-008f,
		.i = 0.0f,
		.d = 9.99999975e-005f,
		.alpha = 0.001f,  // Set this based on how much filtering you want
		.limit = 0.005f,
		.target = 0.0f
};

float speed;
float electricalAngle;
float motorAngleFull;
float diaboloAngleFull;
float diaboloDistance;
float error;




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

MadgwickStruct madgwick;





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

		//pid.integral += pid.error * pid.i;

		//pid.integral = LIMIT(-pid.limit, pid.integral, pid.limit);

		// PID output
		pid.output = pid.p * pid.error + pid.derivative;// + pid.integral;

		pid.output = LIMIT(-pid.limit, pid.output, pid.limit);

		pid.prev_error = pid.error;

		//phaseVoltage = pid.output;

		speed += pid.output;

		speed = LIMIT(-pid.limit, speed, pid.limit);

		electricalAngle += speed;

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
	float distance;
	uint32_t duration;
	float startOffset;
	int direction;
};

struct Movement movement = {
    .start = 0,
    .running = 0,
    .startTimestamp = 0,
    .duration = 3000000,
    .startOffset = 0.0f,
	.direction = 1
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



// Quintic curve function [0-1] in [0-1] out
float quinticCurve(float x) {
    return 10 * pow(x, 3) - 15 * pow(x, 4) + 6 * pow(x, 5);
}

// Quintic integral function [0-1] in [0-0.5] out
float quinticIntegral(float x) {
    return pow(x, 6) - 3 * pow(x, 5) + 2.5 * pow(x, 4);
}

// Quantic Curve Based Speed Profile
float quinticSpeedProfile(float currentTime, float rampTime, float cruiseTime, float maxSpeed) {
    // Limit bottom
    if (currentTime < 0) return 0;

    // Acceleration
    if (currentTime < rampTime) {
        return maxSpeed * quinticCurve(currentTime / rampTime);
    }

    // Constant velocity
    else if (currentTime < rampTime + cruiseTime) {
        return maxSpeed;
    }

    // Deceleration
    else if (currentTime < 2 * rampTime + cruiseTime) {
        return maxSpeed * (1 - quinticCurve((currentTime - rampTime - cruiseTime) / rampTime));
    }

    // Limit top
    else {
        return 0;
    }
}

// Quantic Curve Based Speed Profile Position Integral
float quinticPositionProfile(float currentTime, float rampTime, float cruiseTime, float maxSpeed) {

    // Limit bottom
    if (currentTime < 0) return 0;

    // Acceleration
    if (currentTime < rampTime) {
        return quinticIntegral(currentTime / rampTime) * rampTime * maxSpeed;
    }

    // Constant velocity
    else if (currentTime < rampTime + cruiseTime) {
        return 0.5 * maxSpeed * rampTime + (currentTime - rampTime) * maxSpeed;
    }

    // Deceleration
    else if (currentTime < 2 * rampTime + cruiseTime) {
        return maxSpeed * rampTime + cruiseTime * maxSpeed - (quinticIntegral((2 * rampTime + cruiseTime - currentTime) / rampTime) * rampTime * maxSpeed);
    }

    // Limit top
    else {
        return maxSpeed * rampTime + cruiseTime * maxSpeed;
    }
}





uint8_t ch;
uint8_t size;

void sniffNRF() {
    for (ch = 0; ch <= MAX_CHANNEL; ch++) {
        for (size = MIN_PAYLOAD; size <= MAX_PAYLOAD; size++) {
            configNRF(101, 4);
            //printf("Scanning: CH=%u, PAYLOAD=%u\r\n", ch, size);

            uint32_t start = HAL_GetTick();
            while ((HAL_GetTick() - start) < 1000) {  // 1 second per combo
                if (NRF_DataReady()) {
                    NRF_GetData(buffer);
                    //printf("Packet on CH %u [%u bytes]: ", ch, size);
                    for (uint8_t i = 0; i < size; i++) {
                        asm("nop");//printf("%02X ", buffer[i]);
                    }
                    //printf("\r\n");
                }
            }
        }
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




//	configNRF(10);


	while (1)  {

		sniffNRF();
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
