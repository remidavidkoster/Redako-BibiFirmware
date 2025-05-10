#ifndef INC_SETTINGS_H_
#define INC_SETTINGS_H_

#include "movement.h"



// StartMovement:
//   start delay [ms]
//   direction [LEFT, RIGHT]
//   acceleration distance [m]
//   acceleration angle [0°-60°]
//   coast distance [m]
//   deceleration angle [0°-60°]




void CUE_Start(uint8_t BIBI_NUMBER, uint32_t cue) {
    if (BIBI_NUMBER == 1) {
        if (cue == 1) startMovement((struct MovementStep){0, LEFT, 0.4, 45, 1.2, 45});
        if (cue == 2);
        if (cue == 3);
        if (cue == 4);
    }
    if (BIBI_NUMBER == 2) {
        if (cue == 1);
        if (cue == 2) startMovement((struct MovementStep){0, RIGHT, 0.4, 45, 0.6, 45});
        if (cue == 3);
        if (cue == 4);
    }
    if (BIBI_NUMBER == 3) {
        if (cue == 1);
        if (cue == 2) startMovement((struct MovementStep){1, LEFT, 0.4, 45, 0.6, 45});
        if (cue == 3);
        if (cue == 4);
    }
    if (BIBI_NUMBER == 4) {
        if (cue == 1);
        if (cue == 2) startMovement((struct MovementStep){8, RIGHT, 0.5, 45, 0.5, 45});
        if (cue == 3);
        if (cue == 4);
    }
}

#endif
