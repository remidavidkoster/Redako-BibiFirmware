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


#endif /* INC_DEBUG_H_ */
