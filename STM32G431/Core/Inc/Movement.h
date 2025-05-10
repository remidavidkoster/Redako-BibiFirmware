#ifndef INC_MOVEMENT_H_
#define INC_MOVEMENT_H_


enum {
	LEFT = -1,
	RIGHT = 1,
};


enum {
	ACCELERATING,
	COASTING,
	DECELERATING,
	STOPPING
};


struct Movement {
	uint32_t start;
	uint32_t running;
	uint32_t startTimestamp;
	float accDistance;
	float accAngle;
	float coastDistance;
	float decAngle;
	float startOffset;
	int direction;
	int step;
};


struct Movement movement = {
		.start = 0,
		.running = 0,
		.startTimestamp = 0,
		.startOffset = 0.0f,
		.direction = 1

};


void startMovement(uint32_t delayMs, int dir, float accDist, int accAng, float coastDist, int decAng) {
	if (delayMs > 0) {
		HAL_Delay(delayMs);
	}
	movement.direction = dir;
	movement.accDistance = accDist;
	movement.accAngle = accAng;
	movement.coastDistance = coastDist;
	movement.decAngle = decAng;
	movement.start = 1;
}



#endif
