#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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
#include "Radio.h"
#include "encoder.h"

void SystemClock_Config(void);









typedef enum {
	CUE_CONTROLLED,
	REMOTE_CONTROLLED
} BIBI_Mode_t;

BIBI_Mode_t BIBI_Mode = CUE_CONTROLLED;




















volatile float motorSpeed;
float electricalAngleTarget, electricalAngle;
float motorAngleFullDeg;
float diaboloAngleFullDeg;
float diaboloPosition;
float lastDiaboloPosition;
volatile float diaboloSpeed;
float lastDiaboloSpeed;
float diaboloAcceleration;

float SPEED_ALPHA = 0.01f;
float ACCEL_ALPHA = 0.005f;





/// PID Stuff

typedef struct {
	float p, i, d;
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
    float reset_threshold;

	float on;
} PIDController;


volatile PIDController PID_WeightAngleWithMotorSpeed = {
		.p = 0.000060f,
		.i = 0.0f,
		.d = 0.000015f,
		.alpha = 0.001f,
		.limit = 30.0f,
		.target = 0.0f,
		.reset_threshold = 5.0f
};

volatile PIDController PID_MotorPositionWithVoltage = {
		.p = 15.0f,
		.i = 0.0f,
		.d = 0.5f,
		.alpha = 0.01f,
		.limit = 6.5f,
		.target = 0.0f,
		.reset_threshold = 10000.0f,

		.on = 1
};








/// Main 10kHz pidSpeed loop



float runPID(volatile PIDController *pid, float currentValue) {
	const float dt = 1.0f / SAMPLE_FREQUENCY; // 0.0001 seconds

	pid->error = currentValue - pid->target;

	// Should this be here? What if the target constantly changes?
	if (fabsf(pid->target - pid->lastTarget) > pid->reset_threshold) {
		pid->prev_error = pid->error;
		pid->lastTarget = pid->target;
	}

	// Integral term with gain applied directly
	pid->integral += pid->i * pid->error * dt;

	// Integral windup protection
	pid->integral = LIMIT(-pid->limit, pid->integral, pid->limit);

	// Derivative with low-pass filter
	pid->derivative = pid->alpha * pid->d * (pid->error - pid->prev_error) / dt + (1.0f - pid->alpha) * pid->derivative;

	// PID output (currently PD only)
	pid->output = pid->p * pid->error + pid->integral + pid->derivative;

	// Clamp output
	pid->output = LIMIT(-pid->limit, pid->output, pid->limit);

	// Store error for next iteration
	pid->prev_error = pid->error;

	return pid->output;
}

// Clip PID to 1/8 turn
#define PID_CLIP (TWO_PI / 8.0f * POLE_PAIRS)






/// Timing stuff

unsigned long microsPerReading, microsPrevious, microsUsed;



#define ANGLE_PD_COMP_FACTOR 1.3333333333f



int8_t right;
int8_t backwards;




// Main loop
int main(void) {

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_TIM1_Init();
	MX_ADC1_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_ADC2_Init();
	MX_TIM6_Init();
	MX_SPI2_Init();
	MX_SPI1_Init();
	MX_SPI3_Init();
	MX_TIM4_Init();
	MX_TIM8_Init();



	// Start microsecond timer, overflows after 71 minutes.
	HAL_TIM_Base_Start(&htim2);


	ADC_Init();


	// RGB Led PWM Channels
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);




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
	if (1){//!HAL_GPIO_ReadPin(BUT2_GPIO_Port, BUT2_Pin)){
		BIBI_Mode = REMOTE_CONTROLLED;
	}

	// Keep itself on
	HAL_GPIO_WritePin(SELF_TURN_ON_GPIO_Port, SELF_TURN_ON_Pin, (GPIO_PinState)1);





	// Get Bibi ID
	BIBI_Number = BIBI_GetID();


	//ICM42670 Init, etup rate & scale
	icm42670_init(&imu, ICM42670_DEFAULT_ADDRESS, &hspi1);
	icm42670_mclk_on(&imu);
	icm42670_start_accel(&imu, ICM42670_ACCEL_FS_2G, ICM42670_ODR_1600_HZ);
	icm42670_start_gyro(&imu, ICM42670_GYRO_FS_2000_DPS, ICM42670_ODR_1600_HZ);



	if (BIBI_Mode == REMOTE_CONTROLLED) {
		configNRFSyma();
		while (!HAL_GPIO_ReadPin(BUT2_GPIO_Port, BUT2_Pin));
	} else {
		configNRFTCMfx();

		//		PID_AngleWithSpeed.on = 1;
		//		phaseVoltage = 2;
		PID_WeightAngleWithMotorSpeed.target = 0;
	}



	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	ENC_Setup();



	// Enable motor stuff
	HAL_GPIO_WritePin(MOT_ENABLE_GPIO_Port, MOT_ENABLE_Pin, (GPIO_PinState)1);

	// Motor PWM Enable
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	// Initialize sine lookup table
	initSinTable();



	// Wait until stable, get gyro offsets
	waitForStableGetGyroOffsets();

	// Turn motor on
	MOT_SetPhaseVoltage(5.65f, _3PI_2);

	//	// Wait until stable again, get gyro offsets again
	//	waitForStableGetGyroOffsets();
	HAL_Delay(2000);

	// Update encoder position
	ENC_Update();

	// And save as zero
	MOT_ZeroElectricAngle = normalizeAngle((float)(POLE_PAIRS * ENC_LastAngleRad));

	// Turn motor off
	MOT_SetPhaseVoltage(0, MOT_ZeroElectricAngle);

	// Set electrical angle target to current position
	electricalAngleTarget = ENC_LastFullAngleRad * POLE_PAIRS;








	// Begin Madwick filter at 10000hz
	filter.begin(SAMPLE_FREQUENCY);

	// initialize variables to pace updates to correct rate
	microsPerReading = 1000000 / SAMPLE_FREQUENCY;
	microsPrevious = TIM2->CNT;

	// Turn white led off
	HAL_GPIO_WritePin(LED_NOFF_GPIO_Port, LED_NOFF_Pin, (GPIO_PinState)0);

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





			// Read accelerometer and gyro data
			sensorXYZFloat gyro_data;
			imu_data.accel = icm42670_read_accel_gyro(&imu, &gyro_data);
			imu_data.gyro = gyro_data;

			imu_data.gyroZerod.x = imu_data.gyro.x - gyro_offsets[0];
			imu_data.gyroZerod.y = imu_data.gyro.y - gyro_offsets[1];
			imu_data.gyroZerod.z = imu_data.gyro.z - gyro_offsets[2];

			// Update the Madgwick filter with new IMU values. Coordinate system is translated to have roll align with the Z axis
			filter.updateIMU(imu_data.gyroZerod.z, imu_data.gyroZerod.y, -imu_data.gyroZerod.x, imu_data.accel.z, imu_data.accel.y, -imu_data.accel.x);

			// Get the roll angle
			madgwick.currentAngleDeg = filter.getRoll();
			madgwick.angleDelta = madgwick.currentAngleDeg - madgwick.anglePrev;
			madgwick.anglePrev = madgwick.currentAngleDeg;

			// Detect wrap-around and update turn counter
			if      (madgwick.angleDelta >  180.0f) madgwick.turns--; // Rotated backwards across 0°
			else if (madgwick.angleDelta < -180.0f) madgwick.turns++; // Rotated forward across 360°

			// Compute total angle
			madgwick.angleFullDeg = madgwick.currentAngleDeg + 360.0f * madgwick.turns;








			// Debug movements started 15 seconds after startup. Disabled when moved = 1. Enabled when moved = 0.
			static int moved = 1;
			if (!moved && TIM2->CNT > 15000000){
				moved = 1;
				queueMovement((struct MovementStep){LEFT,  1.0, 90, 0.1, 90}, 0);
				queueMovement((struct MovementStep){RIGHT, 1.0, 90, 0.1, 90}, 0);
				queueMovement((struct MovementStep){LEFT,  1.0, 90, 0.1, 90}, 0);
				queueMovement((struct MovementStep){RIGHT, 1.0, 90, 0.1, 90}, 0);
				queueMovement((struct MovementStep){LEFT,  1.0, 90, 0.1, 90}, 0);
				queueMovement((struct MovementStep){RIGHT, 1.0, 90, 0.1, 90}, 0);
			}





			if (BIBI_Mode == CUE_CONTROLLED){

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
					PID_WeightAngleWithMotorSpeed.target = LIMIT(-90, -movement.accAngle * movement.direction, 90) * ANGLE_PD_COMP_FACTOR;
					PID_WeightAngleWithMotorSpeed.on = 1;
				}

				if (movement.running) {
					float positionDelta = (diaboloPosition - movement.startOffset) * movement.direction;

					if (movement.step == ACCELERATING && positionDelta > movement.accDistance) {
						movement.step = COASTING;
						PID_WeightAngleWithMotorSpeed.target = -2 * movement.direction;
					}

					if (movement.step == COASTING && positionDelta > (movement.accDistance + movement.coastDistance)) {
						movement.step = DECELERATING;
						PID_WeightAngleWithMotorSpeed.target = LIMIT(-90, movement.decAngle * movement.direction, 90) * ANGLE_PD_COMP_FACTOR;
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
							PID_WeightAngleWithMotorSpeed.target = 0;
							movement.stoppingTimestamp = TIM2->CNT;
						}
					}

					if (movement.step == STOPPING && TIM2->CNT - movement.stoppingTimestamp >= 500000) {
						movement.running = 0;
						PID_WeightAngleWithMotorSpeed.on = 0;
						movement.endTimestamp = TIM2->CNT;
					}
				}


				// If we haven't had a message in 5 seconds, reset the last cue started (for debugging purposes)
				if (TIM2->CNT - NRF_ReceiveTimestamp > 5000000){
					lastCueStarted = 0;
				}


				if (NRF_DataReady()) {
					NRF_GetData(buffer);
					NRF_ReceiveInterval = TIM2->CNT - NRF_ReceiveTimestamp;
					NRF_ReceiveTimestamp = TIM2->CNT;


					if ((buffer[0] == REMOTE_V1 || buffer[0] == REMOTE_V2) && (buffer[2] == MODE_TEST || buffer[2] == MODE_FIRE)){
						if (buffer[3] == 1) CUE_Start(BIBI_Number, 1);
						if (buffer[3] == 2) CUE_Start(BIBI_Number, 2);
						if (buffer[3] == 4) CUE_Start(BIBI_Number, 3);
						if (buffer[3] == 8) CUE_Start(BIBI_Number, 4);

						// Shut down Bibi if all buttons are pressed at once
						if (buffer[3] == 15) HAL_GPIO_WritePin(SELF_TURN_ON_GPIO_Port, SELF_TURN_ON_Pin, (GPIO_PinState)0);
					}
				}
			}





			else if (BIBI_Mode == REMOTE_CONTROLLED){

				// If we haven't had a message in a second, turn off the motor
				if (TIM2->CNT - NRF_ReceiveTimestamp > 1000000){
					PID_WeightAngleWithMotorSpeed.on = 0;
					phaseVoltage = 0;
				}

				// If we got a new message
				if (NRF_DataReady()) {
					NRF_GetData(buffer);
					NRF_ReceiveTimestamp = TIM2->CNT;

					right = fix_joystick(buffer[3]);
					backwards = fix_joystick(buffer[1]);



					if (BIBI_Number == 6) PID_WeightAngleWithMotorSpeed.target = (-backwards * 0.8f - right * 0.4f) * 60.0f / 127.0f;
					if (BIBI_Number == 7) PID_WeightAngleWithMotorSpeed.target = (backwards * 0.8f - right * 0.4f) * 60.0f / 127.0f;

					if (BIBI_Number == 8) PID_WeightAngleWithMotorSpeed.target = right * 120.0f / 127.0f;


					PID_WeightAngleWithMotorSpeed.on = 1;
					//					phaseVoltage = 5;
					//					if (ABS(PID_AngleWithSpeed.target) > 45) phaseVoltage = 6;

					// Reset speed if right shoulder button is pressed
					if (buffer[6] & 0b01000000) motorSpeed = 0;
				}
			}





			ENC_Update();






			//Its not diabolospeed. It's motor speed translated to diabolospeed. Not correct. Fix!'

			if (PID_WeightAngleWithMotorSpeed.on){
				motorSpeed += runPID(&PID_WeightAngleWithMotorSpeed, madgwick.angleFullDeg);

				motorSpeed = LIMIT(-PID_WeightAngleWithMotorSpeed.limit, motorSpeed, PID_WeightAngleWithMotorSpeed.limit);

				// Speed should be in meters per second
				electricalAngleTarget += motorSpeed * POLE_PAIRS / (float)SAMPLE_FREQUENCY;
			}
			else {
				motorSpeed = 0;
			}







			if (electricalAngleTarget - ENC_LastFullAngleRad * POLE_PAIRS > PID_CLIP) electricalAngleTarget = ENC_LastFullAngleRad * POLE_PAIRS + PID_CLIP;
			if (ENC_LastFullAngleRad * POLE_PAIRS - electricalAngleTarget > PID_CLIP) electricalAngleTarget = ENC_LastFullAngleRad * POLE_PAIRS - PID_CLIP;



			PID_MotorPositionWithVoltage.target = electricalAngleTarget;

			phaseVoltage = -runPID(&PID_MotorPositionWithVoltage, ENC_LastFullAngleRad * POLE_PAIRS);

			electricalAngle = normalizeAngle((float)POLE_PAIRS * ENC_LastAngleRad - MOT_ZeroElectricAngle);

			if (PID_WeightAngleWithMotorSpeed.on){
				MOT_SetPhaseVoltage(phaseVoltage, electricalAngle);
			}
			else {
				MOT_SetPhaseVoltage(0, electricalAngle);
			}





			motorAngleFullDeg = electricalAngleTarget / POLE_PAIRS * RAD2DEG;

			// Compute the angle the diabolo has made from its startup position
			diaboloAngleFullDeg = motorAngleFullDeg + madgwick.angleFullDeg;

			diaboloPosition = diaboloAngleFullDeg / 360.0f * DIABOLO_CIRCUMFERENCE;

			diaboloSpeed = SPEED_ALPHA * (diaboloPosition - lastDiaboloPosition) * (float)SAMPLE_FREQUENCY + (1.0f - SPEED_ALPHA) * diaboloSpeed;

			diaboloAcceleration = ACCEL_ALPHA * (diaboloSpeed - lastDiaboloSpeed) * (float)SAMPLE_FREQUENCY + (1.0f - ACCEL_ALPHA) * diaboloAcceleration;

			lastDiaboloPosition = diaboloPosition;
			lastDiaboloSpeed = diaboloSpeed;




			// Print debug data

			myData.a = ENC_LastFullAngleRad;
			myData.b = angleTimer / 65535.0f * 4.0f * TWO_PI;
			myData.c = 0;
			myData.d = 0;
			myData.e = 0;
			myData.f = 0;

			printFloats(myData.a, myData.b, myData.c, myData.d, myData.e, myData.f);


			// Update timing variables
			microsUsed = TIM2->CNT - microsPrevious - microsPerReading;

			microsPrevious = microsPrevious + microsPerReading;
		}
	}
}


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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
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
