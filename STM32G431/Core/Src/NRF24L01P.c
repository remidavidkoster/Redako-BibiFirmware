#include "NRF24L01P.h"

#include "stm32g4xx.h"
#include "spi.h"

#define mirf_ADDR_LEN  5
#define mirf_CONFIG ((1<<EN_CRC) | (0<<CRCO))


// In sending mode.
uint8_t PTX;

// Channel 0 - 127 or 0 - 84 in the US.
uint8_t channel;

// Payload width in uint8_ts default 16 max 32.
uint8_t payload;

uint8_t softSpiTransfer(uint8_t txByte) {

//		HAL_SPI_DeInit(&hspi2);
//		hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;   // or HIGH
//		hspi2.Init.CLKPhase    = SPI_PHASE_1EDGE;    // or 2EDGE
//		HAL_SPI_Init(&hspi2);

		uint8_t rxByte = 0x00;    // Variable to store received byte

	HAL_SPI_TransmitReceive(&hspi2, &txByte, &rxByte, 1, HAL_MAX_DELAY);

	return rxByte;
}


//
//uint8_t softSpiTransfer(uint8_t shOut) {
//	uint8_t shIn = 0;
//	for (int i = 0; i < 8; i++) {
//		// Data high / low
//		if (shOut > 127) outHigh();//NRF_MOSI_GPIO_Port->BSRR = NRF_MOSI_Pin;
//		else outLow();//NRF_MOSI_GPIO_Port->BSRR = NRF_MOSI_Pin << 16;
//
//		shIn += readInPin();//((NRF_MISO_GPIO_Port->IDR & NRF_MISO_Pin) != (uint32_t)GPIO_PIN_RESET);
//
//		// Clock high / low
//		clkHigh();//NRF_SCK_GPIO_Port->BSRR = NRF_SCK_Pin;
//		asm("nop");
//		asm("nop");
//		asm("nop");
//		asm("nop");
//		asm("nop");
//		asm("nop");
//		asm("nop");
//		asm("nop");
//		asm("nop");
//		asm("nop");
//		asm("nop");
//		asm("nop");
//		clkLow();//NRF_SCK_GPIO_Port->BSRR = NRF_SCK_Pin << 16;
//		asm("nop");
//		asm("nop");
//		asm("nop");
//		asm("nop");
//		asm("nop");
//		asm("nop");
//		asm("nop");
//		asm("nop");
//		asm("nop");
//		asm("nop");
//		asm("nop");
//		asm("nop");
//
//		shIn <<= (i != 7);
//		shOut <<= 1;
//	}
//	return shIn;
//}


void transferSync(uint8_t *dataout, uint8_t *datain, uint8_t len) {
	uint8_t i;
	for (i = 0; i < len; i++) {
		datain[i] = softSpiTransfer(dataout[i]);
	}
}

void transmitSync(uint8_t *dataout, uint8_t len) {
	uint8_t i;
	for (i = 0; i < len; i++) {
		softSpiTransfer(dataout[i]);
	}
}

// Initializes pins to communicate with the MiRF module
// Should be called in the early initializing phase at startup.
void NRF_Init() {
	ceLow();
	csnHigh();
}


// Constants for RF power
#define RF_PWR_NEG18DBM  0x00
#define RF_PWR_NEG12DBM  0x02
#define RF_PWR_NEG6DBM   0x04
#define RF_PWR_0DBM      0x06

// Constants for data rate
#define RF_DR_250KBPS    0x20
#define RF_DR_1MBPS      0x00
#define RF_DR_2MBPS      0x08


// Sets the important registers in the MiRF module and powers the module
// in receiving mode
// NB: channel and payload must be set now.
void NRF_Config() {
	// Set RF channel
	configRegister(RF_CH, channel);

	// Set length of incoming payload
	configRegister(RX_PW_P0, payload);
	configRegister(RX_PW_P1, payload);
	configRegister(EN_AA, 0); // No auto ack?
	configRegister(SETUP_RETR, 0B00000000); // No retransmits?


//    configRegister(RF_SETUP, RF_DR_250KBPS | RF_PWR_0DBM);
    configRegister(RF_SETUP, 0x27);
	// Start receiver
	powerUpRx();
	flushRx();
}

// Sets the receiving address
void setRADDR(uint8_t * adr) {
	ceLow();
	writeRegister(RX_ADDR_P1, adr, mirf_ADDR_LEN);
	ceHigh();
}

// Sets the transmitting address
void setTADDR(uint8_t * adr) {
	// RX_ADDR_P0 must be set to the sending addr for auto ack to work.

	writeRegister(RX_ADDR_P0, adr, mirf_ADDR_LEN);
	writeRegister(TX_ADDR, adr, mirf_ADDR_LEN);
}

// Checks if data is available for reading
bool NRF_DataReady() {

	// See note in getData() function - just checking RX_DR isn't good enough
	uint8_t status = getStatus();

	// We can short circuit on RX_DR, but if it's not set, we still need
	// to check the FIFO for any pending packets
	if (status & (1 << RX_DR)) return 1;
	return !rxFifoEmpty();
}

bool rxFifoEmpty() {
	uint8_t fifoStatus = 0;

	readRegister(FIFO_STATUS, &fifoStatus, sizeof(fifoStatus));
	return (fifoStatus & (1 << RX_EMPTY));
}



void NRF_GetData(uint8_t * data) {
	// Reads payload uint8_ts into data array

	csnLow();                               // Pull down chip select
	softSpiTransfer(R_RX_PAYLOAD);            // Send cmd to read rx payload
	transferSync(data, data, payload); // Read payload
	csnHigh();                               // Pull up chip select
	// NVI: per product spec, p 67, note c:
	// "The RX_DR IRQ is asserted by a new packet arrival event. The procedure
	// for handling this interrupt should be: 1) read payload through SPI,
	// 2) clear RX_DR IRQ, 3) read FIFO_STATUS to check if there are more
	// payloads available in RX FIFO, 4) if there are more data in RX FIFO,
	// repeat from step 1)."
	// So if we're going to clear RX_DR here, we need to check the RX FIFO
	// in the dataReady() function
	configRegister(STATUS, (1 << RX_DR)); // Reset status register
}

// Clocks only one uint8_t into the given MiRF register
void configRegister(uint8_t reg, uint8_t value) {
	csnLow();
	softSpiTransfer(W_REGISTER | (REGISTER_MASK & reg));
	softSpiTransfer(value);
	csnHigh();
}


// Reads an array of uint8_ts from the given start position in the MiRF registers.
void readRegister(uint8_t reg, uint8_t * value, uint8_t len) {
	csnLow();
	softSpiTransfer(R_REGISTER | (REGISTER_MASK & reg));
	transferSync(value, value, len);
	csnHigh();
}

// Writes an array of uint8_ts into inte the MiRF registers.
void writeRegister(uint8_t reg, uint8_t * value, uint8_t len) {
	csnLow();
	softSpiTransfer(W_REGISTER | (REGISTER_MASK & reg));
	transmitSync(value, len);
	csnHigh();
}


// Sends a data package to the default address. Be sure to send the correct
// amount of uint8_ts as configured as payload on the receiver.
void NRF_Send(uint8_t *value) {
	uint8_t status = 0;

	while (PTX) {
		status = getStatus();

		if ((status & ((1 << TX_DS)  | (1 << MAX_RT)))) {
			PTX = 0;
			break;
		}
	}                  // Wait until last paket is send

	ceLow();

	powerUpTx();                   // Set to transmitter mode , Power up

	csnLow();                      // Pull down chip select
	softSpiTransfer(FLUSH_TX);     // Write cmd to flush tx fifo
	csnHigh();                     // Pull up chip select

	csnLow();                      // Pull down chip select
	softSpiTransfer(W_TX_PAYLOAD); // Write cmd to write payload
	transmitSync(value, payload);  // Write payload
	csnHigh();                     // Pull up chip select

	ceHigh();                      // Start transmission
}

// Test if chip is still sending.
// When sending has finished return chip to listening.
bool NRF_IsSending() {
	uint8_t status;
	if (PTX) {
		status = getStatus();

		// If sending successful (TX_DS) or max retries exceded (MAX_RT).
		if ((status & ((1 << TX_DS)  | (1 << MAX_RT)))) {
			powerUpRx();
			return false;
		}
		return true;
	}
	return false;
}

uint8_t getStatus() {
	uint8_t rv = 0;
	readRegister(STATUS, &rv, 1);
	return rv;
}

void powerUpRx() {
	PTX = 0;
	ceLow();
	configRegister(CONFIG, mirf_CONFIG | ((1 << PWR_UP) | (1 << PRIM_RX)));
	ceHigh();
	configRegister(STATUS, (1 << TX_DS) | (1 << MAX_RT));
}

void flushRx() {
	csnLow();
	softSpiTransfer(FLUSH_RX);
	csnHigh();
}

void powerUpTx() {
	PTX = 1;
	configRegister(CONFIG, mirf_CONFIG | ((1 << PWR_UP) | (0 << PRIM_RX)));
}

void ceHigh() {
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET);
}

void ceLow() {
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);
}

void csnHigh() {
	HAL_GPIO_WritePin(NRF_SS_GPIO_Port, NRF_SS_Pin, GPIO_PIN_SET);
}

void csnLow() {
	HAL_GPIO_WritePin(NRF_SS_GPIO_Port, NRF_SS_Pin, GPIO_PIN_RESET);
}

//void clkHigh() {
//  HAL_GPIO_WritePin(NRF_SCK_GPIO_Port, NRF_SCK_Pin, GPIO_PIN_SET);
//}
//
//void clkLow() {
//  HAL_GPIO_WritePin(NRF_SCK_GPIO_Port, NRF_SCK_Pin, GPIO_PIN_RESET);
//}
//
//
//void outHigh() {
//  HAL_GPIO_WritePin(NRF_MOSI_GPIO_Port, NRF_MOSI_Pin, GPIO_PIN_SET);
//}
//
//void outLow() {
//  HAL_GPIO_WritePin(NRF_MOSI_GPIO_Port, NRF_MOSI_Pin, GPIO_PIN_RESET);
//}
//
//uint8_t readInPin() {
//  return HAL_GPIO_ReadPin(NRF_MISO_GPIO_Port, NRF_MISO_Pin);
//}

void NRF_PowerDown() {
	ceLow();
	configRegister(CONFIG, mirf_CONFIG);
}

void setDataStuff(uint8_t dataStuff) {
	configRegister(RF_SETUP, dataStuff);
}
