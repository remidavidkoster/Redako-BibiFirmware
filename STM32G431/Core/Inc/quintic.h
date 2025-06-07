/*
 * quintic.h
 *
 *  Created on: May 25, 2025
 *      Author: Red
 */

#ifndef INC_QUINTIC_H_
#define INC_QUINTIC_H_


/// Quintic Movement Stuff

typedef struct {
    float rampRatio; // Determines max acceleration used
    float maxSpeed;
    float totalDistance;

    float rampTime;
    float cruiseTime;
    float totalTime;
    float rampDistance;
    float cruiseDistance;
} PositionProfile;

PositionProfile p = {
    .rampRatio = 0.1f,
    .maxSpeed = 0.1f,
    .totalDistance = 0.2f,
    .rampTime = 0.0f,
    .cruiseTime = 0.0f,
    .totalTime = 0.0f,
    .rampDistance = 0.0f,
    .cruiseDistance = 0.0f
};



void distanceSpeedRampRatioToProfileTimes(PositionProfile &p){
    // Calculate ramp up / ramp down time
    p.rampTime = p.maxSpeed / p.rampRatio;

    // Calculate total distance spent ramping up and down
    p.rampDistance = p.rampTime * p.maxSpeed;

    // Check if we can accelerate fast enough to reach max Speed
    if (p.rampDistance <= p.totalDistance){

        // We can! Calculate distance to spend cruising
        p.cruiseDistance = p.totalDistance - p.rampDistance;

        // Calculate time spent cruising
        p.cruiseTime = p.cruiseDistance / p.maxSpeed;
    }

    // If we can't reach max speed
    else {
        // We're fucked. Gotta recalculate the trajectory

        // The ramp ratio determines acceleration. That is fixed. Max speed becomes irrelevant if we can't reach that.
        // With a ramp ratio of 1 we can reach 1 speed in 1 time, and we'd move 0.5 distance. In total we'd move 1 distance in 2 time.
        // So for a ramp ratio of 1, the ramp time is 1 if we need to reach 1 distance.
        // For 4 distance, with a ramp ratio of 1, we'd reach 2 speed in 2 time. Moving 2 distance (0.5 * 2 * 2). Ramp time of 2.
        // With a ramp ratio of 2, we'd reach 2 speed in 1 time. Moving 1 distance in each half. Total distance of 2. Ramp time of 1.

        p.rampTime = sqrtf(p.totalDistance / p.rampRatio);
        p.maxSpeed = sqrtf(p.totalDistance * p.rampRatio);

        p.cruiseDistance = 0;
        p.cruiseTime = 0;
    }

    p.totalTime = 2 * p.rampTime + p.cruiseTime;
}



// Quintic curve function [0-1] in [0-1] out
float quinticCurve(float x) {
    float x2 = x * x;
    float x3 = x2 * x;
    float x4 = x3 * x;
    float x5 = x4 * x;
    return 10.0f * x3 - 15.0f * x4 + 6.0f * x5;
}

// Quintic integral function [0-1] in [0-0.5] out
float quinticIntegral(float x) {
    float x2 = x * x;
    float x3 = x2 * x;
    float x4 = x3 * x;
    float x5 = x4 * x;
    float x6 = x5 * x;
    return x6 - 3.0f * x5 + 2.5f * x4;
}

// Quantic Curve Based Speed Profile
float quinticSpeedProfile(float currentTime, float rampTime, float cruiseTime, float maxSpeed) {
    // Limit bottom
    if (currentTime < 0) return 0;

    // Acceleration
    if (currentTime < rampTime) {
        return maxSpeed * quinticCurve(currentTime / rampTime);
    }

    // Constant velocity
    else if (currentTime < rampTime + cruiseTime) {
        return maxSpeed;
    }

    // Deceleration
    else if (currentTime < 2 * rampTime + cruiseTime) {
        return maxSpeed * (1 - quinticCurve((currentTime - rampTime - cruiseTime) / rampTime));
    }

    // Limit top
    else {
        return 0;
    }
}

// Quantic Curve Based Speed Profile Position Integral
float quinticPositionProfile(float currentTime, float rampTime, float cruiseTime, float maxSpeed) {

    // Limit bottom
    if (currentTime < 0) return 0;

    // Acceleration
    if (currentTime < rampTime) {
        return quinticIntegral(currentTime / rampTime) * rampTime * maxSpeed;
    }

    // Constant velocity
    else if (currentTime < rampTime + cruiseTime) {
        return 0.5 * maxSpeed * rampTime + (currentTime - rampTime) * maxSpeed;
    }

    // Deceleration
    else if (currentTime < 2 * rampTime + cruiseTime) {
        return maxSpeed * rampTime + cruiseTime * maxSpeed - (quinticIntegral((2 * rampTime + cruiseTime - currentTime) / rampTime) * rampTime * maxSpeed);
    }

    // Limit top
    else {
        return maxSpeed * rampTime + cruiseTime * maxSpeed;
    }
}




#endif /* INC_QUINTIC_H_ */
