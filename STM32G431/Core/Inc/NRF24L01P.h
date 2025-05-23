#ifndef NRF24L01P_H_
#define NRF24L01P_H_

#include "main.h"
#include "stdbool.h"

extern uint8_t mirf_CONFIG;

// Constants for RF power
#define RF_PWR_NEG18DBM  0x00
#define RF_PWR_NEG12DBM  0x02
#define RF_PWR_NEG6DBM   0x04
#define RF_PWR_0DBM      0x06

// Constants for data rate
#define RF_DR_250KBPS    0x20
#define RF_DR_1MBPS      0x00
#define RF_DR_2MBPS      0x08

/* Memory Map */
#define CONFIG 0x00
#define EN_AA 0x01
#define EN_RXADDR 0x02
#define SETUP_AW 0x03
#define SETUP_RETR 0x04
#define RF_CH 0x05
#define RF_SETUP 0x06
#define STATUS 0x07
#define OBSERVE_TX 0x08
#define CD 0x09
#define RX_ADDR_P0 0x0A
#define RX_ADDR_P1 0x0B
#define RX_ADDR_P2 0x0C
#define RX_ADDR_P3 0x0D
#define RX_ADDR_P4 0x0E
#define RX_ADDR_P5 0x0F
#define TX_ADDR 0x10
#define RX_PW_P0 0x11
#define RX_PW_P1 0x12
#define RX_PW_P2 0x13
#define RX_PW_P3 0x14
#define RX_PW_P4 0x15
#define RX_PW_P5 0x16
#define FIFO_STATUS 0x17

/* Bit Mnemonics */
#define MASK_RX_DR 6
#define MASK_TX_DS 5
#define MASK_MAX_RT 4
#define EN_CRC 3
#define CRCO 2
#define PWR_UP 1
#define PRIM_RX 0
#define ENAA_P5 5
#define ENAA_P4 4
#define ENAA_P3 3
#define ENAA_P2 2
#define ENAA_P1 1
#define ENAA_P0 0
#define ERX_P5 5
#define ERX_P4 4
#define ERX_P3 3
#define ERX_P2 2
#define ERX_P1 1
#define ERX_P0 0
#define AW 0
#define ARD 4
#define ARC 0
#define PLL_LOCK 4
#define RF_DR 3
#define RF_PWR 1
#define LNA_HCURR 0
#define RX_DR 6
#define TX_DS 5
#define MAX_RT 4
#define RX_P_NO 1
#define TX_FULL 0
#define PLOS_CNT 4
#define ARC_CNT 0
#define TX_REUSE 6
#define FIFO_FULL 5
#define TX_EMPTY 4
#define RX_FULL 1
#define RX_EMPTY 0

/* Instruction Mnemonics */
#define R_REGISTER 0x00
#define W_REGISTER 0x20
#define REGISTER_MASK 0x1F
#define R_RX_PAYLOAD 0x61
#define W_TX_PAYLOAD 0xA0
#define FLUSH_TX 0xE1
#define FLUSH_RX 0xE2
#define REUSE_TX_PL 0xE3
#define NOP 0xFF

/* Data Stuff */
#define HIGHPOWER250K 0B00100110
#define LOWPOWER250K 0B00100000
#define HIGHPOWER2M 0B00001110
#define LOWPOWER2M 0B00001000


/// NRF24L01 stuff

// In sending mode.
extern uint8_t PTX;

// Channel 0 - 127 or 0 - 84 in the US.
extern uint8_t channel;

// Payload width in uint8_ts default 16 max 32.
extern uint8_t payload;

#ifdef __cplusplus
extern "C" {
#endif

uint8_t softSpiTransfer(uint8_t working);
void NRF_Init();
void NRF_Config(uint8_t rfSetup);
void NRF_Send(uint8_t *value);
void setRADDR(uint8_t * adr);
void setTADDR(uint8_t * adr);
bool NRF_DataReady();
bool NRF_IsSending();
bool rxFifoEmpty();
bool txFifoEmpty();
void NRF_GetData(uint8_t * data);
uint8_t getStatus();

void transmitSync(uint8_t *dataout, uint8_t len);
void transferSync(uint8_t *dataout, uint8_t *datain, uint8_t len);
void configRegister(uint8_t reg, uint8_t value);
void readRegister(uint8_t reg, uint8_t * value, uint8_t len);
void writeRegister(uint8_t reg, uint8_t * value, uint8_t len);
void powerUpRx();
void powerUpTx();
void NRF_PowerDown();
void setDataStuff(uint8_t dataStuff);

void csnHigh();
void csnLow();

void ceHigh();
void ceLow();

void clkHigh();

void clkLow();


void outHigh();

void outLow();

uint8_t readInPin();




void flushRx();

#ifdef __cplusplus
}
#endif


#endif /* NRF24L01P_H_ */
