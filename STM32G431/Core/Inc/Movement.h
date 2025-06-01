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
	float newPosition;
	float maxSpeed;
	float acceleration;
	float startOffset;
	int direction;
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
    float newPosition;
    float maxSpeed;
    float acceleration;
    uint32_t startTime;   // Delay before this movement starts [s]
};

#define MAX_QUE_LENGTH 100

MovementStep queuedMovements[MAX_QUE_LENGTH];
uint8_t queuedMovementCount;
uint8_t lastCueStarted;


void startMovement(struct MovementStep step) {
    movement.newPosition = step.newPosition;
    movement.maxSpeed = step.maxSpeed;
    movement.acceleration = step.acceleration;
    movement.start = 1;
}





void queueMovement(struct MovementStep step, float delay){
	step.startTime = TIM2->CNT + 1000000 * delay;
	queuedMovements[queuedMovementCount] = step;
	queuedMovementCount++;
}




#endif
