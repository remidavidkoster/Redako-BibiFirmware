/*
 * CRC.h
 *
 *  Created on: Jun 14, 2025
 *      Author: Red
 */

#ifndef CRC_H_
#define CRC_H_

void CRC_Init_16_ARC(){
	// Enable CRC peripheral clock
	RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;

	// Reset CRC peripheral to ensure clean state
	RCC->AHB1RSTR |= RCC_AHB1RSTR_CRCRST;
	RCC->AHB1RSTR &= ~RCC_AHB1RSTR_CRCRST;

	// Configure CRC peripheral for CRC-16-IBM
	// CRC_CR register configuration:
	// - POLYSIZE = 01 (16-bit polynomial)
	// - REV_IN = 11 (reflect input bytes)
	// - REV_OUT = 1 (reflect output)
	// - RESET = 0 (don't reset, we'll do it manually)
	CRC->CR = (0x1 << CRC_CR_POLYSIZE_Pos) |    // 16-bit polynomial
			(0x3 << CRC_CR_REV_IN_Pos) |      // Reflect input by byte
			(0x1 << CRC_CR_REV_OUT_Pos);      // Reflect output

	// Set polynomial (0x8005 for CRC-16-IBM)
	CRC->POL = 0x8005;

	// Set initial value (0x0000 for CRC-16-IBM)
	CRC->INIT = 0x0000;

	// Reset CRC calculation
	CRC->CR |= CRC_CR_RESET;
}


uint16_t CRC_Calculate(uint8_t *data, uint32_t length) {
	// Reset CRC calculation
	CRC->CR |= CRC_CR_RESET;

	// Feed data to CRC engine
	for (uint32_t i = 0; i < length; i++) {
		*((volatile uint8_t*)&CRC->DR) = data[i];
	}

	// Return 16-bit result
	return (uint16_t)(CRC->DR & 0xFFFF);
}


#endif /* CRC_H_ */
