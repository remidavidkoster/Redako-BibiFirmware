/*
 * motorMath.h
 *
 *  Created on: May 10, 2025
 *      Author: Red
 */

#ifndef INC_MOTORMATH_H_
#define INC_MOTORMATH_H_


#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define LIMIT(low,x,high) ((x)<(low)?(low):((x)>(high)?(high):(x)))
#define ABS(x) ((x) < 0 ? -(x) : (x))



#define POLE_PAIRS 7
#define _1_DIV_POLE_PAIRS 0.14285714285714285714f


#define DIABOLO_CIRCUMFERENCE 0.408
#define METERS2RAD (TWO_PI / DIABOLO_CIRCUMFERENCE)
#define RAD2METERS (DIABOLO_CIRCUMFERENCE / TWO_PI)

/// Sine math stuff
#define TABLE_SIZE 2048


#define TWO_PI 6.28318530718f  // 2 * π
#define _SQRT3_2 0.86602540378f
#define _1_OVER_2PI 0.15915494309f  // 1 / (2 * PI)
#define _3PI_2 4.71238898038f
#define RAD2DEG 57.29577951308232087680f
#define DEG2RAD 0.01745329251994329577f
#define PI 3.141592653589793f

float sinTable[TABLE_SIZE];

void initSinTable() {
	for (int i = 0; i < TABLE_SIZE; i++) {
		sinTable[i] = sinf((TWO_PI * i) / TABLE_SIZE);
	}
}

// Fast sine lookup. Divide gets precomputed here
inline float fastSin(float x) {
	int index = (int)((x * (TABLE_SIZE / TWO_PI))) & (TABLE_SIZE - 1);
	return sinTable[index];
}

// Fast cosine using phase shift
inline float fastCos(float x) {
	return fastSin(x + (TWO_PI / 4));  // cos(x) = sin(x + π/2)
}

// Single SIN/COS function for FOC
inline void _sincos(float a, float* s, float* c){
	*s = fastSin(a);
	*c = fastCos(a);
}

// normalizing radian angle to [0,2PI]
inline float _normalizeAngle(float angle){
	float norm = angle - TWO_PI * ((int)(angle * _1_OVER_2PI));
	return (norm < 0) ? (norm + TWO_PI) : norm;
}



/// PWM / Motor Stuff

#define SUPPLY_VOLTAGE 11.3f

#define PWM_MAX ((int)8499)

#define _1_DIV_SUPPLY_VOLTAGExPWM_MAX (1/SUPPLY_VOLTAGE*PWM_MAX)

// Arbitrary PWM limits for now
int lowerPWMLimit = 10;
int upperPWMLimit = 8489;

int motorNumber = 0;

// Default open loop phase voltage
float phaseVoltage = 0;

// Current phase voltages Ua,Ub and Uc set to motor [V]
float Ua, Ub, Uc;

// Main FOC Function
void setPhaseVoltage(float Uq, float angle_el) {
	float Ualpha, Ubeta, center, _ca,_sa;

	// Sinusoidal PWM modulation
	// Inverse Park + Clarke transformation
	_sincos(angle_el, &_sa, &_ca);

	// Inverse park transform
	Ualpha =  -_sa * Uq;  // -sin(angle) * Uq;
	Ubeta =  _ca * Uq;    //  cos(angle) * Uq;

	// Clarke transform
	Ua = Ualpha;
	Ub = -0.5f * Ualpha + _SQRT3_2 * Ubeta;
	Uc = -0.5f * Ualpha - _SQRT3_2 * Ubeta;

	center = SUPPLY_VOLTAGE * 0.5f;

	if (1){//foc_modulation == 1){
		// Midpoint Clamp
		float Umin = min(Ua, min(Ub, Uc));
		float Umax = max(Ua, max(Ub, Uc));
		center -= (Umax+Umin) * 0.5f;
	}

	// Add center voltages [V]
	Ua += center;
	Ub += center;
	Uc += center;


	// calculate duty cycle and PWM value in one go. limited in [0,1]
	uint16_t dc_a = Ua * _1_DIV_SUPPLY_VOLTAGExPWM_MAX;
	uint16_t dc_b = Ub * _1_DIV_SUPPLY_VOLTAGExPWM_MAX;
	uint16_t dc_c = Uc * _1_DIV_SUPPLY_VOLTAGExPWM_MAX;

	// set the voltages in driver
	uint16_t A = LIMIT(lowerPWMLimit, dc_a, upperPWMLimit);
	uint16_t B = LIMIT(lowerPWMLimit, dc_b, upperPWMLimit);
	uint16_t C = LIMIT(lowerPWMLimit, dc_c, upperPWMLimit);

	if (motorNumber == 0 || motorNumber == 1){
		TIM1->CCR1 = A;
		TIM1->CCR2 = B;
		TIM1->CCR3 = C;
	}
	if (motorNumber == 0 || motorNumber == 2){
		TIM8->CCR1 = A;
		TIM8->CCR2 = B;
		TIM8->CCR3 = C;
	}
}


//	setPhaseVoltage(5.65, _3PI_2);
//	HAL_Delay(4000);
//	ENC_Update();
//	zero_electric_angle = _normalizeAngle((float)(POLE_PAIRS * angle_prev));
//	setPhaseVoltage(0, _3PI_2);

#endif /* INC_MOTORMATH_H_ */
