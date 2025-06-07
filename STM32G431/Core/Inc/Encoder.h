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


float ENC_LastAngleRad;
double ENC_LastFullAngleRad;




float angleTimer;

float MOT_ZeroElectricAngle;

int32_t ENC_FullRotations=0; // full rotation tracking

const int32_t sensor_direction = 1;

#define _2PI 6.28318530718f



uint8_t spiTransferENC(uint8_t txByte) {
    // Wait until TXE (Transmit buffer empty)
    while (!(SPI3->SR & SPI_SR_TXE));
    *(volatile uint8_t *)&SPI3->DR = txByte;

    // Wait until RXNE (Receive buffer not empty)
    while (!(SPI3->SR & SPI_SR_RXNE));
    return *(volatile uint8_t *)&SPI3->DR;
}


void ENC_Update(){
	HAL_GPIO_WritePin(SPI3_CSN_GPIO_Port, SPI3_CSN_Pin, (GPIO_PinState)0);

	txData[0] = 0b1010 << 4;
	txData[1] = 0x03;

    for (int i = 0; i < 6; i++) {
    	rxData[i] = spiTransferENC(txData[i]);
    }
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

	ENC_LastFullAngleRad = (double)ENC_FullRotations * _2PI + ENC_LastAngleRad;

	angleTimer = TIM4->CNT;
}

void ENC_Setup(){

	// Start encoder timer
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

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
