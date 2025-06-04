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

void NRF_ConfigCueButtonControlled() {
	// NRF24L01P init
	NRF_Init();
	setRADDR((uint8_t *)"TCMfx");
	setTADDR((uint8_t *)"TCMfx");
	payload = TCMPAYLOADSIZE;
	channel = 101;
	NRF_Config(RF_DR_2MBPS | RF_PWR_0DBM);
}



#pragma pack(push, 1)
typedef struct {
	uint8_t command;
	uint16_t bibiNumber;
	uint16_t newPosition;
	uint8_t maxSpeed;
	uint8_t acceleration;
} MotionCommand;
#pragma pack(pop)



enum {
	COMMAND_MOVE_LEFT_SIDE = 10,
	COMMAND_MOVE_RIGHT_SIDE = 11,
	COMMAND_SHUTDOWN = 20
};


void NRF_ConfigMotionControlled() {
	// NRF24L01P init
	NRF_Init();
	setRADDR((uint8_t *)"TCMfx");
	setTADDR((uint8_t *)"TCMfx");
	payload = sizeof(MotionCommand);
	channel = 101;
	NRF_Config(RF_DR_2MBPS | RF_PWR_0DBM);
}















uint32_t NRF_ReceiveTimestamp;
uint32_t NRF_ReceiveInterval;






#endif /* INC_RADIO_H_ */
