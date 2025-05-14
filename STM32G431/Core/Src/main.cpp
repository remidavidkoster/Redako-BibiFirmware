#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "spi.h"

#include <string.h>
#include "ICM-42670-P.h"
#include "NRF24L01P.h"
#include "math.h"
#include "MadgwickAHRS.h"
#include "settings.h"
#include "Syma.h"
#include "motorMath.h"
#include "IMU.h"
#include "BibiSupport.h"
#include "Debug.h"
#include "UID.h"

void SystemClock_Config(void);


// Identification number of this bibi
uint8_t BIBI_Number;




/// Radio Stuff

// NRF buffer
#define TCMPAYLOADSIZE 4

typedef struct __attribute__((packed)) {
	uint8_t what;
	uint8_t who;
	uint8_t maxSpeed;          // per 0.01
	uint8_t accelerationRatio; // per 0.01

} ControlData;

uint8_t buffer[SYMAPAYLOADSIZE];
ControlData txData;
ControlData rxData;

#define CONTROL_BIBI 10
#define START_CUE 14

#define MODE_TEST 150
#define MODE_FIRE 200





// Debug send command
uint8_t send;

void configNRFTCMfx() {
	// NRF24L01P init
	NRF_Init();
	setRADDR((uint8_t *)"TCMfx");
	setTADDR((uint8_t *)"TCMfx");
	payload = TCMPAYLOADSIZE;
	channel = 101;
	NRF_Config(RF_DR_2MBPS | RF_PWR_NEG12DBM);
}



uint32_t NRF_ReceiveTimestamp;
uint32_t NRF_ReceiveInterval;









typedef enum {
    CUE_CONTROLLED,
    REMOTE_CONTROLLED
} BIBI_Mode_t;

BIBI_Mode_t BIBI_Mode = CUE_CONTROLLED;








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
		.p = 0.000002f,
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




/// Timing stuff

unsigned long microsPerReading, microsPrevious, microsUsed;





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



	// RGB Led PWM Channels
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

	// If second button is pressed during startup, this will be remote controlled
	if (!HAL_GPIO_ReadPin(BUT2_GPIO_Port, BUT2_Pin)){
		BIBI_Mode = REMOTE_CONTROLLED;
	}

	// Keep itself on
	HAL_GPIO_WritePin(SELF_TURN_ON_GPIO_Port, SELF_TURN_ON_Pin, (GPIO_PinState)1);


	BIBI_Number = BIBI_GetID();


	//ICM42670 Init, etup rate & scale
	icm42670_init(&imu, ICM42670_DEFAULT_ADDRESS, &hi2c2);
	icm42670_mclk_on(&imu);
	icm42670_start_accel(&imu, ICM42670_ACCEL_FS_2G, ICM42670_ODR_1600_HZ);
	icm42670_start_gyro(&imu, ICM42670_GYRO_FS_2000_DPS, ICM42670_ODR_1600_HZ);



	if (BIBI_Mode == REMOTE_CONTROLLED) {
	    configNRFSyma();
	    while (!HAL_GPIO_ReadPin(BUT2_GPIO_Port, BUT2_Pin));
	} else {
	    configNRFTCMfx();

		pid.on = 0;
		phaseVoltage = 2;
		pid.target = 0;
	}





	// Enable motor stuff
	HAL_GPIO_WritePin(DRIVER_ENABLE1_GPIO_Port, DRIVER_ENABLE1_Pin, (GPIO_PinState)1);
	HAL_GPIO_WritePin(BOOST_ENABLE_GPIO_Port, BOOST_ENABLE_Pin, (GPIO_PinState)1);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	// Initialize sine lookup table
	initSinTable();

	// Wait until stable and get gyro offsets
	waitForStableGetGyroOffsets();





	// Begin Madwick filter at 1000hz
	filter.begin(SAMPLE_FREQUENCY);

	// initialize variables to pace updates to correct rate
	microsPerReading = 1000000 / SAMPLE_FREQUENCY;
	microsPrevious = TIM2->CNT;

	// Start main motor interrupt
	HAL_TIM_Base_Start_IT(&htim6);


	while (1)  {

		// If another loop is due
		if (TIM2->CNT - microsPrevious >= microsPerReading) {

			// Update global battery Voltage
			BAT_Update();

			// Display Battery on led
			BAT_VoltageToRGB(BAT.voltage);

			// Run charge management logic
			CHG_RunLogic();

			// Low battery shut down
			if (TIM2->CNT > 1000000){
				BAT_CheckLowShutdown();
			}

			// Button shut down
			if (!HAL_GPIO_ReadPin(BUT2_GPIO_Port, BUT2_Pin)) HAL_GPIO_WritePin(SELF_TURN_ON_GPIO_Port, SELF_TURN_ON_Pin, (GPIO_PinState)0);





			// Read accelerometer and gyro data for IMU B (6.7mm offset above point of rotation. X+ is down. Y+ to the is right.)
			imu_data.accel = icm42670_read_accel(&imu);
			imu_data.gyro = icm42670_read_gyro(&imu);

			imu_data.gyroZerod.x = imu_data.gyro.x - gyro_offsets[0];
			imu_data.gyroZerod.y = imu_data.gyro.y - gyro_offsets[1];
			imu_data.gyroZerod.z = imu_data.gyro.z - gyro_offsets[2];

			// Update the Madgwick filter with new IMU values. Coordinate system is translated to have roll align with the Z axis
			filter.updateIMU(imu_data.gyroZerod.z, -imu_data.gyroZerod.x, -imu_data.gyroZerod.y, imu_data.accel.z, -imu_data.accel.x, -imu_data.accel.y);

			// Calculate the roll angle. Down is 90 degrees (on hardware V1.0). Subtract this for now.
			madgwick.currentAngle = filter.getRoll() - 90.0f < -180.0f ? filter.getRoll() + 270.0f : filter.getRoll() - 90.0f;
			madgwick.angleDelta = madgwick.currentAngle - madgwick.anglePrev;
			madgwick.anglePrev = madgwick.currentAngle;

			// Detect wrap-around and update turn counter (Commented out now that we again don't have any feedback)
			//			if      (madgwick.angleDelta >  180.0f) madgwick.turns--; // Rotated backwards across 0°
			//			else if (madgwick.angleDelta < -180.0f) madgwick.turns++; // Rotated forward across 360°

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





			// Debug movements started 15 seconds after startup. Disabled when moved = 1. Enabled when moved = 0.
			static int moved = 1;
			if (!moved && TIM2->CNT > 15000000){
				moved = 1;
				queueMovement((struct MovementStep){LEFT,  0.5, 30, 0.5, 30}, 0);
				queueMovement((struct MovementStep){RIGHT, 0.5, 60, 0.5, 60}, 10);
				queueMovement((struct MovementStep){LEFT,  0.5, 30, 0.5, 30}, 20);
				queueMovement((struct MovementStep){RIGHT, 0.5, 60, 0.5, 60}, 20);
				queueMovement((struct MovementStep){LEFT,  0.5, 30, 0.5, 30}, 20);
				queueMovement((struct MovementStep){RIGHT, 0.5, 60, 0.5, 60}, 20);
			}





			// If we still have cued movements, and there's currently none running
			if (queuedMovementCount && !movement.running){

				// If it's time for the next one
				if (TIM2->CNT > queuedMovements[0].startTime){

					// Start next cued movement
					startMovement(queuedMovements[0]);

					// Move cues down a row
				    memmove(&queuedMovements[0], &queuedMovements[1], sizeof(struct MovementStep) * (MAX_QUE_LENGTH - 1));

				    // Zero out the last element
				    memset(&queuedMovements[MAX_QUE_LENGTH - 1], 0, sizeof(struct MovementStep));

				    // Decrement qued movement counter
				    queuedMovementCount--;
				}
			}


			if (movement.start){
				movement.start = 0;
				movement.startTimestamp = TIM2->CNT;
				movement.running = 1;
				movement.step = ACCELERATING;
				movement.startOffset = diaboloPosition;
                pid.target = -movement.accAngle * movement.direction;
				phaseVoltage = 5;
				pid.on = 1;
			}

			if (movement.running) {
				float positionDelta = (diaboloPosition - movement.startOffset) * movement.direction;

				if (movement.step == ACCELERATING && positionDelta > movement.accDistance) {
					movement.step = COASTING;
					pid.target = -2 * movement.direction;
				}

				if (movement.step == COASTING && positionDelta > (movement.accDistance + movement.coastDistance)) {
					movement.step = DECELERATING;
					pid.target = movement.decAngle * movement.direction;
				}

				if (movement.step == DECELERATING && (movement.direction * diaboloSpeed) < 0.1f) {

					// If another movement is due, stop this one right away
					if (queuedMovementCount && TIM2->CNT > queuedMovements[0].startTime){
						movement.running = 0;
						movement.endTimestamp = TIM2->CNT;
					}

					// Otherwise move to the 'stopping' step, where it waits for half a second until it stabilizes
					else {
						movement.step = STOPPING;
						pid.target = 0;
						movement.stoppingTimestamp = TIM2->CNT;
					}
				}

				if (movement.step == STOPPING && TIM2->CNT - movement.stoppingTimestamp >= 500000) {
					movement.running = 0;
					phaseVoltage = 2;
					pid.on = 0;
					movement.endTimestamp = TIM2->CNT;
				}
			}





			if (BIBI_Mode == REMOTE_CONTROLLED){

				// If we haven't had a message in a second, turn off the motor
				if (TIM2->CNT - NRF_ReceiveTimestamp > 1000000){
					pid.on = 0;
					phaseVoltage = 0;
				}



				// If we got a new message
				if (NRF_DataReady()) {
					NRF_GetData(buffer);
					NRF_ReceiveTimestamp = TIM2->CNT;
					pid.target = -fix_joystick(buffer[3]) * 60.0f / 127.0f;
					pid.on = 1;
					phaseVoltage = 5;
					if (ABS(pid.target) > 45) phaseVoltage = 6;

					// Reset speed if right shoulder button is pressed
					if (buffer[6] & 0b01000000) speed = 0;
				}
			}


			else {


				// If we haven't had a message in 5 seconds, reset the last cue started (for debugging purposes)
				if (TIM2->CNT - NRF_ReceiveTimestamp > 5000000){
					lastCueStarted = 0;
				}



				if (NRF_DataReady()) {
					NRF_GetData(buffer);
					NRF_ReceiveInterval = TIM2->CNT - NRF_ReceiveTimestamp;
					NRF_ReceiveTimestamp = TIM2->CNT;


					if (buffer[0] == START_CUE && (buffer[2] == MODE_TEST || buffer[2] == MODE_FIRE)){
						if (buffer[3] == 1) CUE_Start(BIBI_Number, 1);
						if (buffer[3] == 2) CUE_Start(BIBI_Number, 2);
						if (buffer[3] == 4) CUE_Start(BIBI_Number, 3);
						if (buffer[3] == 8) CUE_Start(BIBI_Number, 4);
					}
				}
			}


			// Debug send command
			if (send == 1) {
				send = 0;

				NRF_Send(buffer);
				while (NRF_IsSending());
			}


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
