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
#include "quintic.h"

void SystemClock_Config(void);









typedef enum {
	CUE_CONTROLLED,
	REMOTE_CONTROLLED
} BIBI_Mode_t;

BIBI_Mode_t BIBI_Mode = CUE_CONTROLLED;




















float motorSpeedTarget;
double motorAngleFullDeg;

double electricalAngleTarget;
double electricalAngle;

double diaboloAngleFullDeg;
double diaboloPosition;
double lastDiaboloPosition;

float diaboloSpeed;
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

	double target;
	double lastTarget;
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
		.reset_threshold = 5.0f,

		.on = 1
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



volatile PIDController PID_BibiSpeedWithWeightAngle = {
		.p = 60.0f,//100.0f,
		.i = 0.0f,//40.0f,
		.d = 0.0f,
		.alpha = 0.01f,
		.limit = 120.0f,
		.target = 0.0f,
		.reset_threshold = 10000.0f,

		.on = 1
};




volatile PIDController PID_BibiPositionWithBibiSpeed = {
		.p = -0.5f,
		.i = 0.0f,
		.d = 0.0f,
		.alpha = 0.01f,
		.limit = 5.0f,
		.target = 0.0f,
		.reset_threshold = 10000.0f,

		.on = 1
};










float positionTarget;
float lastPositionTarget;

float speedTarget;
float lastSpeedTarget;

float accelerationTarget;
float accelerationTargetFiltered;

/// Main 10kHz pidSpeed loop



float runPID(volatile PIDController *pid, double currentValue) {
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

	// PID output
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


// -- Constants --
float accelerationTargetAlpha = 0.0004f;         // Filter smoothing factor
float accelerationFeedForwardGain = 0;//-80.0f;           // Feedforward gain


// Main loop
int main(void) {

	// Reset of all peripherals, Initializes the Flash interface and the Systick.
	HAL_Init();

	// Configure the system clock
	SystemClock_Config();

	// Initialize all configured peripherals
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


	// Initialize ADC stuff
	ADC_Init();

	// Initialize RGB Led stuff
	RGB_Init();




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





	// Get Bibi ID
	BIBI_Number = BIBI_GetID();

	// Force remote controlled for steering Bibi halves now
	if (BIBI_Number == 6) BIBI_Mode = REMOTE_CONTROLLED;
	if (BIBI_Number == 7) BIBI_Mode = REMOTE_CONTROLLED;



	// Initialize IMU
	IMU_Init();



	// Config radio
	if (BIBI_Mode == REMOTE_CONTROLLED) {
		NRF_ConfigSyma();

		// Wait for mode setting button to be depressed
		while (!HAL_GPIO_ReadPin(BUT2_GPIO_Port, BUT2_Pin));
	}

	else if (BIBI_Mode == CUE_CONTROLLED){
		NRF_ConfigTCMfx();
	}



	// Setup magnetic encoder
	ENC_Setup();



	// Initialize and enable motor driver
	MOT_Init();
	MOT_Enable();



	// Wait until stable, get gyro offsets
	IMU_WaitForStableGetGyroOffsets();

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


	// Movement defaults for debugging
	p.totalDistance = 1;
	p.rampRatio = 1;
	p.maxSpeed = 0.5f;




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
			BAT_CheckLowShutdown();

			// Check button shutdown
			SYS_ButShutdown();

			// Get IMU data and update Madgwick filter
			MAD_Update();

			// Check if the led button is pressed
			RGB_CheckFadexButton();




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

					// Set the last target as offset, so we won't slowly drift.
					movement.startOffset = lastPositionTarget;

					// Calculate the motion times from the distance, acceleration, and speed we've set.
					distanceSpeedRampRatioToProfileTimes(p);

					// Turn on the PID
					PID_WeightAngleWithMotorSpeed.on = 1;
				}


				if (movement.running){
					float runTime = (TIM2->CNT - movement.startTimestamp) / 1000000.0f;

					// Calculate position and speed targets
					positionTarget = (movement.startOffset + movement.direction * quinticPositionProfile(runTime, p.rampTime, p.cruiseTime, p.maxSpeed));
					speedTarget = (positionTarget - lastPositionTarget) * SAMPLE_FREQUENCY;
					lastPositionTarget = positionTarget;

					// If we've passed the runtime, stop 'movement' (the PIDs still try to hold the current position)
					if (runTime >= p.totalTime) {
						movement.running = 0;
						movement.direction = -movement.direction;
					}
				}

				// Pass position target to position PID
				PID_BibiPositionWithBibiSpeed.target = positionTarget;

				// Pass calculated speed target + PID result to speed PID
				PID_BibiSpeedWithWeightAngle.target = speedTarget + runPID(&PID_BibiPositionWithBibiSpeed, diaboloPosition);




				// Check radio
				if (NRF_DataReady()) {
					NRF_GetData(buffer);
					NRF_ReceiveInterval = TIM2->CNT - NRF_ReceiveTimestamp;
					NRF_ReceiveTimestamp = TIM2->CNT;

					// Start cues if we have to
					if ((buffer[0] == REMOTE_V1 || buffer[0] == REMOTE_V2) && (buffer[2] == MODE_TEST || buffer[2] == MODE_FIRE)){
						if (buffer[3] == 1) CUE_Start(BIBI_Number, 1);
						if (buffer[3] == 2) CUE_Start(BIBI_Number, 2);
						if (buffer[3] == 4) CUE_Start(BIBI_Number, 3);
						if (buffer[3] == 8) CUE_Start(BIBI_Number, 4);

						// Shut down Bibi if all buttons are pressed at once
						if (buffer[3] == 15) HAL_GPIO_WritePin(SELF_TURN_ON_GPIO_Port, SELF_TURN_ON_Pin, (GPIO_PinState)0);
					}
				}

				// If we haven't had a message in 5 seconds, reset the last cue started (for debugging purposes)
				if (TIM2->CNT - NRF_ReceiveTimestamp > 5000000){
					lastCueStarted = 0;
				}
			}





			else if (BIBI_Mode == REMOTE_CONTROLLED){

				// If we got a new message
				if (NRF_DataReady()) {
					NRF_GetData(buffer);
					NRF_ReceiveTimestamp = TIM2->CNT;

					// If the checksum is correct
					if (buffer[9] == symaChecksum(&buffer[0])){

						right = fix_joystick(buffer[3]);
						backwards = fix_joystick(buffer[1]);

						// Steering Bibi
						if (BIBI_Number == 6) PID_BibiSpeedWithWeightAngle.target = (-backwards + right * (20 + buffer[0]) / 400.0f) / 100.0f;
						if (BIBI_Number == 7) PID_BibiSpeedWithWeightAngle.target = (backwards  + right * (20 + buffer[0]) / 400.0f) / 100.0f;

						// First Bibi V1.1
						if (BIBI_Number == 8) PID_BibiSpeedWithWeightAngle.target = right / 100.0f;
						// if (BIBI_Number == 8) PID_WeightAngleWithMotorSpeed.target = right * 120.0f / 127.0f;


						PID_WeightAngleWithMotorSpeed.on = 1;
					}
				}


				// If we haven't had a message in a second, turn off the motor
				if (TIM2->CNT - NRF_ReceiveTimestamp > 1000000){
					PID_WeightAngleWithMotorSpeed.on = 0;
					phaseVoltage = 0;
				}
			}





			// Calculate and filter acceleration target
			accelerationTarget = (PID_BibiSpeedWithWeightAngle.target - lastSpeedTarget) * SAMPLE_FREQUENCY;
			lastSpeedTarget = PID_BibiSpeedWithWeightAngle.target;

			// Changed this without re-testing. Should be better to apply the feed forward acceleration to the full speed setpoint
			// accelerationTarget = (speedTarget - lastSpeedTarget) * SAMPLE_FREQUENCY;

			// Filter target acceleration (more relevant for remote than for quintic curves
			accelerationTargetFiltered = (1.0f - accelerationTargetAlpha) * accelerationTargetFiltered + accelerationTargetAlpha * accelerationTarget;


			// Add filtered derivative-based feedforward to angle to help drive acceleration
			float angleFeedForward = accelerationFeedForwardGain * accelerationTargetFiltered;

			// Steady state conversion formula didn't seem to help a lot
			// float angleFeedForward = K_ff * computeAngle(accelerationTargetFiltered);


			// Update encoder
			ENC_Update();

			// Set angle target based on acceleration feed forward and diabolo speed PID
			PID_WeightAngleWithMotorSpeed.target = LIMIT(-120, angleFeedForward + runPID(&PID_BibiSpeedWithWeightAngle, diaboloSpeed), 120);

			// Set motor speed target with angle PID
			motorSpeedTarget += runPID(&PID_WeightAngleWithMotorSpeed, madgwick.angleFullDeg);

			// Limit motor speed (might not be necessary anymore with encoders)
			motorSpeedTarget = LIMIT(-PID_WeightAngleWithMotorSpeed.limit, motorSpeedTarget, PID_WeightAngleWithMotorSpeed.limit);

			// Set electrical angle target through desired motor speed
			electricalAngleTarget += motorSpeedTarget * POLE_PAIRS / (float)SAMPLE_FREQUENCY;




			// Clip electrical angle target to 1/8 of a circle if it's gone haywire
			if (electricalAngleTarget - ENC_LastFullAngleRad * POLE_PAIRS > PID_CLIP) electricalAngleTarget = ENC_LastFullAngleRad * POLE_PAIRS + PID_CLIP;
			if (ENC_LastFullAngleRad * POLE_PAIRS - electricalAngleTarget > PID_CLIP) electricalAngleTarget = ENC_LastFullAngleRad * POLE_PAIRS - PID_CLIP;


			// Pass electrical angle target to motor position controller
			PID_MotorPositionWithVoltage.target = electricalAngleTarget;

			// Set phase voltage with motor position controller
			phaseVoltage = -runPID(&PID_MotorPositionWithVoltage, ENC_LastFullAngleRad * POLE_PAIRS);

			// Calculate current electrical angle
			electricalAngle = normalizeAngle((float)POLE_PAIRS * ENC_LastAngleRad - MOT_ZeroElectricAngle);

			// Set phase voltage (if the last PID is turned on)
			MOT_SetPhaseVoltage(PID_WeightAngleWithMotorSpeed.on ? phaseVoltage : 0, electricalAngle);








			/// Compute diabolo stats

			// Full motor angle since startup in degrees
			motorAngleFullDeg = electricalAngleTarget / POLE_PAIRS * RAD2DEG;

			// Full diabolo rotations in degrees since startup
			diaboloAngleFullDeg = motorAngleFullDeg + madgwick.angleFullDeg;

			// Diabolo position [m] since startup
			diaboloPosition = diaboloAngleFullDeg / 360.0f * DIABOLO_CIRCUMFERENCE;

			// Diabolo speed [m/s]
			diaboloSpeed = SPEED_ALPHA * (diaboloPosition - lastDiaboloPosition) * (float)SAMPLE_FREQUENCY + (1.0f - SPEED_ALPHA) * diaboloSpeed;

			// Diabolo acceleration [m/s²]
			diaboloAcceleration = ACCEL_ALPHA * (diaboloSpeed - lastDiaboloSpeed) * (float)SAMPLE_FREQUENCY + (1.0f - ACCEL_ALPHA) * diaboloAcceleration;

			// Save last values
			lastDiaboloPosition = diaboloPosition;
			lastDiaboloSpeed = diaboloSpeed;




			// Print debug data
			myData.a = imu_data.gyroZerod.z;
			myData.b = PID_MotorPositionWithVoltage.error;
			myData.c = PID_MotorPositionWithVoltage.derivative;
			myData.d = ENC_LastFullAngleRad;
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
