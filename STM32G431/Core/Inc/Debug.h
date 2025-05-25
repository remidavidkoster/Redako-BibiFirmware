/*
 * Debug.h
 *
 *  Created on: May 10, 2025
 *      Author: Red
 */

#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_



/// Float send debug stuff

#pragma pack(push, 1)
typedef struct {
	float a;
	float b;
	float c;
	float d;
	float e;
	float f;
} FloatStruct;
#pragma pack(pop)

FloatStruct myData;

float reverseFloatBytes(float input) {
	uint8_t *bytes = (uint8_t *)&input;
	uint8_t reversed[4];

	reversed[0] = bytes[3];
	reversed[1] = bytes[2];
	reversed[2] = bytes[1];
	reversed[3] = bytes[0];

	float output;
	memcpy(&output, reversed, sizeof(float));
	return output;
}

void sendFloats(FloatStruct *data) {
	FloatStruct reversedData;

	reversedData.a = reverseFloatBytes(data->a);
	reversedData.b = reverseFloatBytes(data->b);
	reversedData.c = reverseFloatBytes(data->c);
	reversedData.d = reverseFloatBytes(data->d);
	reversedData.e = reverseFloatBytes(data->e);
	reversedData.f = reverseFloatBytes(data->f);

	HAL_UART_Transmit(&huart2, (uint8_t *)&reversedData, sizeof(FloatStruct), 1000);
}




uint8_t firstSendUart = 1;

void UART2_SendDMA(uint8_t *data, uint16_t size) {
	// Reverse byte order for each float (little-endian to big-endian)
	uint32_t *data_as_int = (uint32_t*)data;
	uint16_t num_floats = size / sizeof(float);

	for (uint16_t i = 0; i < num_floats; ++i) {
		uint32_t value = data_as_int[i];
		uint32_t reversed = 0;
		reversed |= (value & 0xFF) << 24;
		reversed |= (value & 0xFF00) << 8;
		reversed |= (value & 0xFF0000) >> 8;
		reversed |= (value & 0xFF000000) >> 24;
		data_as_int[i] = reversed;
	}

	// Wait for DMA transfer complete flag
	while (!(DMA1->ISR & DMA_ISR_TCIF3) && !firstSendUart); // Channel 3 for USART2 TX

	// Clear the Transfer Complete Flag
	DMA1->IFCR = DMA_IFCR_CTCIF3;

	// Disable DMA channel before configuring
	DMA1_Channel3->CCR &= ~DMA_CCR_EN;

	// Configure DMA transfer
	DMA1_Channel3->CPAR = (uint32_t)&USART2->TDR;  // UART2 TX register
	DMA1_Channel3->CMAR = (uint32_t)data;         // Source buffer
	DMA1_Channel3->CNDTR = size;                  // Data size in bytes

	DMA1_Channel3->CCR = DMA_CCR_MINC             // Memory increment mode
		| DMA_CCR_DIR                             // Memory-to-peripheral
		| DMA_CCR_TCIE                            // Enable transfer complete interrupt if needed
		| DMA_CCR_PL_1;                           // Medium priority

	// Clear pending flags
	DMA1->IFCR = DMA_IFCR_CTCIF3;

	// Enable DMA channel
	DMA1_Channel3->CCR |= DMA_CCR_EN;

	// Enable UART2 DMA transmission
	USART2->CR3 |= USART_CR3_DMAT;
}






float floats[6];

void printFloats(float a) {
	floats[0] = a;
	UART2_SendDMA((uint8_t *)floats, 4);
}

void printFloats(float a, float b) {
	floats[0] = a;
	floats[1] = b;
	UART2_SendDMA((uint8_t *)floats, 8);
}

void printFloats(float a, float b, float c) {
	floats[0] = a;
	floats[1] = b;
	floats[2] = c;
	UART2_SendDMA((uint8_t *)floats, 12);
}

void printFloats(float a, float b, float c, float d) {
	floats[0] = a;
	floats[1] = b;
	floats[2] = c;
	floats[3] = d;
	UART2_SendDMA((uint8_t *)floats, 16);
}

void printFloats(float a, float b, float c, float d, float e) {
	floats[0] = a;
	floats[1] = b;
	floats[2] = c;
	floats[3] = d;
	floats[4] = e;
	UART2_SendDMA((uint8_t *)floats, 20);
}

void printFloats(float a, float b, float c, float d, float e, float f) {
	floats[0] = a;
	floats[1] = b;
	floats[2] = c;
	floats[3] = d;
	floats[4] = e;
	floats[5] = f;
	UART2_SendDMA((uint8_t *)floats, 24);
}








#endif /* INC_DEBUG_H_ */
