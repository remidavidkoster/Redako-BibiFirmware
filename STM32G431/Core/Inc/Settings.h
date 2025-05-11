#ifndef INC_SETTINGS_H_
#define INC_SETTINGS_H_

#include "movement.h"



// StartMovement:
//   direction [LEFT, RIGHT]
//   acceleration distance [m]
//   acceleration angle [0°-60°]
//   coast distance [m]
//   deceleration angle [0°-60°]
//   start delay [s]


// Cue 1
// and then, without WARNING (1:39)

// +20s
// with an air of quiet confidence, the DIABOLO (1:59) moves closer

// Cue 2
// diabolos, unlike handbalancers, travel in FLOCK (3:23)

// +8s
// MORE (3:31) diabolos emerge

// Cue 3
// making her presence appear far larger than it truely IS (5:04)

// Cue 4
// Aylish shooing the last Bibi away


void CUE_Start(uint8_t BIBI_NUMBER, uint32_t cue) {
	if (cue != lastCueStarted){
		lastCueStarted = cue;
		if (BIBI_NUMBER == 1) {
			if (cue == 1) {
				queueMovement((struct MovementStep){LEFT, 0.3, 45, 0.3, 45}, 20);
				queueMovement((struct MovementStep){LEFT, 0.3, 60, 0.3, 60}, 20);
				queueMovement((struct MovementStep){LEFT, 0.3, 45, 0.3, 45}, 20);
				queueMovement((struct MovementStep){RIGHT, 0.3, 45, 0.3, 45}, 20);
				queueMovement((struct MovementStep){RIGHT, 0.6, 45, 0.6, 45}, 20);
				queueMovement((struct MovementStep){LEFT, 0.3, 30, 0.3, 30}, 20);
				queueMovement((struct MovementStep){RIGHT, 0.5, 30, 0.3, 30}, 40);
			}
			if (cue == 2);
			if (cue == 3);
			if (cue == 4) startMovement((struct MovementStep){RIGHT, 5, 60, 5, 45});
		}
		if (BIBI_NUMBER == 2) {
			if (cue == 1);
			if (cue == 2) startMovement((struct MovementStep){RIGHT, 0.4, 45, 0.6, 45});
			if (cue == 3) queueMovement((struct MovementStep){LEFT, 0.4, 20, 0.6, 20}, 0);
			if (cue == 4);
		}
		if (BIBI_NUMBER == 3) {
			if (cue == 1);
			if (cue == 2) queueMovement((struct MovementStep){LEFT, 0.4, 45, 0.6, 45}, 1);
			if (cue == 3) queueMovement((struct MovementStep){RIGHT, 0.4, 20, 0.6, 45}, 40);
			if (cue == 4);
		}
		if (BIBI_NUMBER == 4) {
			if (cue == 1);
			if (cue == 2) queueMovement((struct MovementStep){RIGHT, 0.5, 45, 0.5, 45}, 8);
			if (cue == 3) queueMovement((struct MovementStep){LEFT, 0.5, 20, 0.5, 20}, 50);
			if (cue == 4);
		}
		if (BIBI_NUMBER == 5) {
			if (cue == 1);
			if (cue == 2) queueMovement((struct MovementStep){RIGHT, 0.5, 45, 0.5, 45}, 8);
			if (cue == 3) queueMovement((struct MovementStep){LEFT, 0.5, 20, 0.5, 20}, 50);
			if (cue == 4);
		}
	}
}

#endif
