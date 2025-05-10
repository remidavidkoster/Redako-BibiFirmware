#ifndef INC_SETTINGS_H_
#define INC_SETTINGS_H_

#include "movement.h"

// Identification number of this bibi
#define BIBI_NUMBER 4


// StartMovement:
//   start delay [ms]
//   direction [LEFT, RIGHT]
//   acceleration distance [m]
//   acceleration angle [0°-60°]
//   coast distance [m]
//   deceleration angle [0°-60°]


void CUE_Start(uint32_t cue) {
	if (BIBI_NUMBER == 1) {
		if (cue == 1) startMovement(0, LEFT, 0.4, 45, 1.2, 45);
		if (cue == 2);
		if (cue == 3);
		if (cue == 4);
	}
	if (BIBI_NUMBER == 2) {
		if (cue == 1);
		if (cue == 2) startMovement(0, RIGHT, 0.4, 45, 0.6, 45);
		if (cue == 3);
		if (cue == 4);
	}
	if (BIBI_NUMBER == 3) {
		if (cue == 1);
		if (cue == 2) startMovement(1000, LEFT, 0.4, 45, 0.6, 45);
		if (cue == 3);
		if (cue == 4);
	}
	if (BIBI_NUMBER == 4) {
		if (cue == 1);
		if (cue == 2) startMovement(8000, RIGHT, 0.4, 45, 0.6, 45);
		if (cue == 3);
		if (cue == 4);
	}
}

#endif
