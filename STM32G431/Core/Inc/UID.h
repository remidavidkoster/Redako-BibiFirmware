#ifndef UID_CHECKER_H
#define UID_CHECKER_H

#include "stm32g4xx.h"
#include <stdint.h>
#include <stdio.h>

#define BIBIS 5

static const uint32_t known_uids[BIBIS][3] = {
    {0x21002b, 0x5130500d, 0x2035344b},
    {0x5e003f, 0x5130500c, 0x2035344b},
    {0x210036, 0x5130500d, 0x2035344b},
    {0x1e002e, 0x5130500d, 0x2035344b},
    {0x1f0049, 0x5130500d, 0x2035344b}
};

static inline uint8_t BIBI_GetID(void) {
    uint32_t uid[3];

    // Read the UID directly
    uid[0] = *(uint32_t *)0x1FFF7590;
    uid[1] = *(uint32_t *)0x1FFF7594;
    uid[2] = *(uint32_t *)0x1FFF7598;

    // Check against known UIDs
    for (uint8_t i = 0; i < BIBIS; i++) {
        if (uid[0] == known_uids[i][0] &&
            uid[1] == known_uids[i][1] &&
            uid[2] == known_uids[i][2]) {
            return i + 1;
        }
    }

    return 0; // No match found
}

#endif /* INC_UID_H_ */
