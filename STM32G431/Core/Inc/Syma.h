/*
 * Syma.h
 *
 *  Created on: May 10, 2025
 *      Author: Red
 */

#ifndef INC_SYMA_H_
#define INC_SYMA_H_

#define SYMAPAYLOADSIZE 10 // packet size


uint8_t fixedAylish_rx_tx_addr[] = {0xB5, 0xFC, 0x04, 0x00, 0xA2};
uint8_t fixedSmall_rx_tx_addr[] = {0x88, 0x0, 0x8, 0x0c, 0xa1};
uint8_t bind_rx_tx_addr[] = {0xab, 0xac, 0xad, 0xae, 0xaf};

const uint8_t chans_bind[] = {0x4b, 0x30, 0x40, 0x09}; // bind chan
const uint8_t chans_fixedSmall[] = {0x12, 0x22, 0x32, 0x42}; // bind chan

// Configure the NRF for listening to the Syma remote
void configNRFSyma() {

	// Configure 16 bit checksum
	mirf_CONFIG = ((1 << EN_CRC) | (1 << CRCO));
	NRF_Init();
	setRADDR(fixedSmall_rx_tx_addr);
	setTADDR(fixedSmall_rx_tx_addr);
	payload = SYMAPAYLOADSIZE;
	channel = 0x32;
	NRF_Config(RF_DR_250KBPS | RF_PWR_0DBM);
}



uint8_t symaChecksum(uint8_t *data){
	uint8_t sum = data[0];

	for (int i=1; i < SYMAPAYLOADSIZE-1; i++) sum ^= data[i];

	return sum + 0x55;
}






int8_t fix_joystick(uint8_t raw) {
    return (raw <= 127) ? -(int8_t)raw : (int8_t)(raw - 128);
}




#endif /* INC_SYMA_H_ */
