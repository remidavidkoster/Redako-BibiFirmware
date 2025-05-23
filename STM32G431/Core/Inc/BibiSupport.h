/*
 * BibiSupport.h
 *
 *  Created on: May 10, 2025
 *      Author: Red
 */

#ifndef INC_BIBISUPPORT_H_
#define INC_BIBISUPPORT_H_





/// RGB LED Stuff

#define RGB_PWM_MAX 8499

void RGB_Set(float r, float g, float b){
	TIM8->CCR3 = RGB_PWM_MAX - r * RGB_PWM_MAX;
	TIM8->CCR1 = RGB_PWM_MAX - g * RGB_PWM_MAX;
	TIM8->CCR2 = RGB_PWM_MAX - b * RGB_PWM_MAX;
}

// Voltage thresholds for color transitions
#define VBAT_MIN_VOLTAGE   3.4f  // Red
#define VBAT_MID_VOLTAGE   3.75f  // Green
#define VBAT_MAX_VOLTAGE   4.2f  // Blue


#define BAT_SHUTDOWN_VOLTAGE 3.4f



void BAT_VoltageToRGB(float voltage) {
	float r = 0.0f, g = 0.0f, b = 0.0f;

	if (voltage == 0) {
		RGB_Set(0, 0, 0);
		return;
	}

	// Clamp voltage between min and max
	if (voltage < VBAT_MIN_VOLTAGE) voltage = VBAT_MIN_VOLTAGE;
	if (voltage > VBAT_MAX_VOLTAGE) voltage = VBAT_MAX_VOLTAGE;

	if (voltage <= VBAT_MID_VOLTAGE) {
		// Interpolate from Red (1,0,0) to Green (0,1,0)
		float t = (voltage - VBAT_MIN_VOLTAGE) / (VBAT_MID_VOLTAGE - VBAT_MIN_VOLTAGE);
		r = 1.0f - t;
		g = t;
		b = 0.0f;
	} else {
		// Interpolate from Green (0,1,0) to Blue (0,0,1)
		float t = (voltage - VBAT_MID_VOLTAGE) / (VBAT_MAX_VOLTAGE - VBAT_MID_VOLTAGE);
		r = 0.0f;
		g = 1.0f - t;
		b = t;
	}

	RGB_Set(r, g, b);
}





/// ADC Stuff

struct ADC {
	uint16_t valueBattery;
	uint16_t valueVRefInt;
	uint16_t vrefintCal;
	float vrefintVoltage;
	const float BAT_VOLTAGE_DIVIDER_RATIO = 2.0f;
} ADC;





/// Battery Stuff

struct BAT {
	float voltage;
} BAT;

struct CHG {
	uint8_t charging;
	uint8_t standby;
	uint8_t plugged;
	uint8_t enabled;
} CHG;

void BAT_Update(){
	ADC.valueVRefInt = ADC1->DR;
	ADC.valueBattery = ADC2->DR;
	BAT.voltage = (float)ADC.valueBattery / (float)ADC.valueVRefInt * ADC.vrefintVoltage * ADC.BAT_VOLTAGE_DIVIDER_RATIO;
}


// Low battery shut down

void BAT_CheckLowShutdown(){
	if (BAT.voltage < BAT_SHUTDOWN_VOLTAGE) HAL_GPIO_WritePin(SELF_TURN_ON_GPIO_Port, SELF_TURN_ON_Pin, (GPIO_PinState)0);
}



/// Charge stuff

void CHG_RunLogic(){

	// Charge stuff
	CHG.standby = !HAL_GPIO_ReadPin(STDBY_GPIO_Port, STDBY_Pin);
	CHG.charging = !HAL_GPIO_ReadPin(CHRG_GPIO_Port, CHRG_Pin);
	CHG.plugged = HAL_GPIO_ReadPin(VBUS_PRESENT_GPIO_Port, VBUS_PRESENT_Pin);
	CHG.enabled = BAT.voltage > 2.75f;

	// Charge enable logic
	HAL_GPIO_WritePin(CHARGE_ENABLE_GPIO_Port, CHARGE_ENABLE_Pin, (GPIO_PinState)(CHG.plugged && CHG.enabled));
}





#endif /* INC_BIBISUPPORT_H_ */
