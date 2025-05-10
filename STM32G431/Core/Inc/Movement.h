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
	uint32_t stoppingTimestamp;
	float startDelay;
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





// Structure to hold a single movement
struct MovementStep {
    float delay;         // Delay before this movement starts [s]
    int direction;       // Direction [LEFT, RIGHT]
    float accDistance;   // Acceleration distance [m]
    float accAngle;      // Acceleration angle [0°-60°]
    float coastDistance; // Coast distance [m]
    float decAngle;      // Deceleration angle [0°-60°]
};





void startMovement(struct MovementStep step) {
    movement.startDelay = step.delay;
    movement.direction = step.direction;
    movement.accDistance = step.accDistance;
    movement.accAngle = step.accAngle;
    movement.coastDistance = step.coastDistance;
    movement.decAngle = step.decAngle;
    movement.start = 1;
}



#endif
