/*
 * Radio.h
 *
 *  Created on: May 25, 2025
 *      Author: Red
 */

#ifndef INC_RADIO_H_
#define INC_RADIO_H_






/// Radio Stuff

// NRF buffer
#define TCMPAYLOADSIZE 4



uint8_t buffer[SYMAPAYLOADSIZE];


#define CONTROL_BIBI 10
#define REMOTE_V1 14
#define REMOTE_V2 23

#define MODE_TEST 150
#define MODE_FIRE 200





// Debug send command
uint8_t send;

void NRF_ConfigTCMfx() {
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






#endif /* INC_RADIO_H_ */
