/*
 * Encoder.h
 *
 *  Created on: May 25, 2025
 *      Author: Red
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_


uint8_t txData[6];
uint8_t rxData[6];
uint32_t lastRawAngle = 0;


float velocity=0.0f;
float ENC_LastAngleRad=0.0f; // result of last call to getSensorAngle(), used for full rotations and velocity
float ENC_LastFullAngleRad=0.0f; // result of last call to getSensorAngle(), used for full rotations and velocity
float angleTimer;

float MOT_ZeroElectricAngle;

uint32_t angle_prev_ts=0; // timestamp of last call to getAngle, used for velocity
float vel_angle_prev=0.0f; // angle at last call to getVelocity, used for velocity
uint32_t vel_angle_prev_ts=0; // last velocity calculation timestamp
int32_t ENC_FullRotations=0; // full rotation tracking
int32_t vel_full_rotations=0; // previous full rotation value for velocity calculation

const int32_t sensor_direction = 1;

#define _2PI 6.28318530718f


void ENC_Update(){
	HAL_GPIO_WritePin(SPI3_CSN_GPIO_Port, SPI3_CSN_Pin, (GPIO_PinState)0);

	txData[0] = 0b1010 << 4;
	txData[1] = 0x03;

	HAL_SPI_TransmitReceive(&hspi3, txData, rxData, 6, HAL_MAX_DELAY);

	lastRawAngle = 0;

	// Extract bits from rawData1 and rawData2
	lastRawAngle |= ((uint32_t)(rxData[0 + 2]) << 13);  // Upper 8 bits (ANGLE[20:13])
	lastRawAngle |= ((uint32_t)(rxData[1 + 2]) << 5); // Middle 8 bits (ANGLE[12:5])
	lastRawAngle |= ((uint32_t)(rxData[2 + 2]) & 0b11111000) >> 3; // Lower 5 bits (ANGLE[4:0])

	HAL_GPIO_WritePin(SPI3_CSN_GPIO_Port, SPI3_CSN_Pin, (GPIO_PinState)1);

	float val = (2097151 - lastRawAngle) * 0.00000299605622633914f;
	//	    angle_prev_ts = TIM6->CNT;
	float d_angle = val - ENC_LastAngleRad;
	// if overflow happened track it as full rotation
	if(abs(d_angle) > (0.8f*_2PI) ) ENC_FullRotations += ( d_angle > 0 ) ? -1 : 1;
	ENC_LastAngleRad = val;

	ENC_LastFullAngleRad = (float)ENC_FullRotations * _2PI + ENC_LastAngleRad;

	angleTimer = TIM4->CNT;
}

void ENC_Setup(){
	// Set ABZ resolution to highest setting (4096 ppr / 16384 spr)
	txData[0] = 0b0110 << 4;
	txData[1] = 0x07;
	txData[2] = 0xFF;

	HAL_GPIO_WritePin(SPI3_CSN_GPIO_Port, SPI3_CSN_Pin, (GPIO_PinState)0);
	HAL_SPI_TransmitReceive(&hspi3, txData, rxData, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI3_CSN_GPIO_Port, SPI3_CSN_Pin, (GPIO_PinState)1);

	txData[0] = 0b0110 << 4;
	txData[1] = 0x08;
	txData[2] = 0b11110000;

	HAL_GPIO_WritePin(SPI3_CSN_GPIO_Port, SPI3_CSN_Pin, (GPIO_PinState)0);
	HAL_SPI_TransmitReceive(&hspi3, txData, rxData, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI3_CSN_GPIO_Port, SPI3_CSN_Pin, (GPIO_PinState)1);


}



#endif /* INC_ENCODER_H_ */
