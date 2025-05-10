#ifndef INC_MOVEMENT_H_
#define INC_MOVEMENT_H_


enum {
	LEFT = -1,
	RIGHT = 1,
};

enum {
	WAITING,
	ACCELERATING,
	COASTING,
	DECELERATING,
	STOPPING
};


struct Movement {
	uint32_t start;
	uint32_t running;
	uint32_t startTimestamp;
	uint32_t endTimestamp;
	uint32_t stoppingTimestamp;
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



enum {
	START_TIME,
	CURRENT_TIME,
	END_TIME,
};

// Structure to hold a single movement
struct MovementStep {
    int direction;       // Direction [LEFT, RIGHT]
    float accDistance;   // Acceleration distance [m]
    float accAngle;      // Acceleration angle [0°-60°]
    float coastDistance; // Coast distance [m]
    float decAngle;      // Deceleration angle [0°-60°]
    uint32_t startTime;         // Delay before this movement starts [s]
};

#define MAX_QUEUES 100

MovementStep quedMovements[MAX_QUEUES];
uint8_t quedMovementCount;
uint8_t lastCueStarted;


void startMovement(struct MovementStep step) {
    movement.direction = step.direction;
    movement.accDistance = step.accDistance;
    movement.accAngle = step.accAngle;
    movement.coastDistance = step.coastDistance;
    movement.decAngle = step.decAngle;
    movement.start = 1;
}





void queMovement(struct MovementStep step, float delay){
	step.startTime = TIM2->CNT + 1000000 * delay;
	quedMovements[quedMovementCount] = step;
	quedMovementCount++;
}




#endif
