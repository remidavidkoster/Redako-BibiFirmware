#ifndef UID_CHECKER_H
#define UID_CHECKER_H

#include "stm32g4xx.h"
#include <stdint.h>
#include <stdio.h>

#define BIBIS 11

// Identification number of this bibi
uint8_t BIBI_Number;

static const uint32_t known_uids[BIBIS][3] = {
    {0x2e003f}, // 1
    {0x34002f}, // 2
    {0x320023}, // 3
    {0x35006c}, // 4
    {0x35005f}, // 5
    {0x340059}, // 6
    {0x31004a}, // 7
    {0x320059}, // 8
    {0x330064}, // 9
    {0x34005c}, // 10
    {0x34002d}  // 11
};

static inline uint8_t BIBI_GetID(void) {
    uint32_t uid[3];

    // Read the UID directly
    uid[0] = *(uint32_t *)0x1FFF7590;
//  uid[1] = *(uint32_t *)0x1FFF7594;
//  uid[2] = *(uint32_t *)0x1FFF7598;

    // Check against known UIDs
    for (uint8_t i = 0; i < BIBIS; i++) {
        if (uid[0] == known_uids[i][0]) {
            return i + 1;
        }
    }

    return 1; // In case no match, use Bibi number 1 (used for debugging)
}

#endif /* INC_UID_H_ */
