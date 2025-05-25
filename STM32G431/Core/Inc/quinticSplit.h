/*
 * quinticSplit.h
 *
 *  Created on: May 25, 2025
 *      Author: Red
 */

#ifndef INC_QUINTICSPLIT_H_
#define INC_QUINTICSPLIT_H_

/// Quintic Movement Stuff with Separate Ramp Up/Down Ratios

typedef struct {
    float rampUpRatio;   // Determines max acceleration during ramp up
    float rampDownRatio; // Determines max acceleration during ramp down
    float maxSpeed;
    float totalDistance;

    float rampUpTime;
    float rampDownTime;
    float cruiseTime;
    float totalTime;
    float rampUpDistance;
    float rampDownDistance;
    float cruiseDistance;
} PositionProfile;

PositionProfile p = {
    .rampUpRatio = 0.1f,
    .rampDownRatio = 0.15f,  // Different from ramp up
    .maxSpeed = 0.1f,
    .totalDistance = 0.2f,
    .rampUpTime = 0.0f,
    .rampDownTime = 0.0f,
    .cruiseTime = 0.0f,
    .totalTime = 0.0f,
    .rampUpDistance = 0.0f,
    .rampDownDistance = 0.0f,
    .cruiseDistance = 0.0f
};

void distanceSpeedRampRatioToProfileTimes(PositionProfile &p){
    // Calculate ramp up time
    p.rampUpTime = p.maxSpeed / p.rampUpRatio;

    // Calculate ramp down time
    p.rampDownTime = p.maxSpeed / p.rampDownRatio;

    // Calculate distance spent ramping up
    p.rampUpDistance = p.rampUpTime * p.maxSpeed * 0.5f;

    // Calculate distance spent ramping down
    p.rampDownDistance = p.rampDownTime * p.maxSpeed * 0.5f;

    // Total distance spent in ramps
    float totalRampDistance = p.rampUpDistance + p.rampDownDistance;

    // Check if we can accelerate fast enough to reach max Speed
    if (totalRampDistance <= p.totalDistance){

        // We can! Calculate distance to spend cruising
        p.cruiseDistance = p.totalDistance - totalRampDistance;

        // Calculate time spent cruising
        p.cruiseTime = p.cruiseDistance / p.maxSpeed;
    }

    // If we can't reach max speed
    else {
        // We need to recalculate the trajectory for a triangular profile
        // The peak speed will be less than maxSpeed

        // For a triangular profile: totalDistance = 0.5 * peakSpeed * (rampUpTime + rampDownTime)
        // Where: peakSpeed = rampUpRatio * rampUpTime = rampDownRatio * rampDownTime
        // So: rampUpTime = peakSpeed / rampUpRatio, rampDownTime = peakSpeed / rampDownRatio
        // Substituting: totalDistance = 0.5 * peakSpeed * (peakSpeed/rampUpRatio + peakSpeed/rampDownRatio)
        // Solving for peakSpeed: peakSpeed = sqrt(2 * totalDistance / (1/rampUpRatio + 1/rampDownRatio))

        float peakSpeed = sqrtf(2.0f * p.totalDistance / (1.0f/p.rampUpRatio + 1.0f/p.rampDownRatio));

        p.maxSpeed = peakSpeed;
        p.rampUpTime = peakSpeed / p.rampUpRatio;
        p.rampDownTime = peakSpeed / p.rampDownRatio;

        p.rampUpDistance = p.rampUpTime * peakSpeed * 0.5f;
        p.rampDownDistance = p.rampDownTime * peakSpeed * 0.5f;

        p.cruiseDistance = 0;
        p.cruiseTime = 0;
    }

    p.totalTime = p.rampUpTime + p.cruiseTime + p.rampDownTime;
}

// Quintic curve function [0-1] in [0-1] out
float quinticCurve(float x) {
    return 10 * pow(x, 3) - 15 * pow(x, 4) + 6 * pow(x, 5);
}

// Quintic integral function [0-1] in [0-0.5] out
float quinticIntegral(float x) {
    return pow(x, 6) - 3 * pow(x, 5) + 2.5 * pow(x, 4);
}

// Quintic Curve Based Speed Profile with separate ramp ratios
float quinticSpeedProfile(float currentTime, float rampUpTime, float rampDownTime, float cruiseTime, float maxSpeed) {
    // Limit bottom
    if (currentTime < 0) return 0;

    // Acceleration
    if (currentTime < rampUpTime) {
        return maxSpeed * quinticCurve(currentTime / rampUpTime);
    }

    // Constant velocity
    else if (currentTime < rampUpTime + cruiseTime) {
        return maxSpeed;
    }

    // Deceleration
    else if (currentTime < rampUpTime + cruiseTime + rampDownTime) {
        float decelerationProgress = (currentTime - rampUpTime - cruiseTime) / rampDownTime;
        return maxSpeed * (1 - quinticCurve(decelerationProgress));
    }

    // Limit top
    else {
        return 0;
    }
}

// Quintic Curve Based Position Profile with separate ramp ratios
float quinticPositionProfile(float currentTime, float rampUpTime, float rampDownTime, float cruiseTime, float maxSpeed) {

    // Limit bottom
    if (currentTime < 0) return 0;

    // Acceleration phase
    if (currentTime < rampUpTime) {
        return quinticIntegral(currentTime / rampUpTime) * rampUpTime * maxSpeed;
    }

    // Constant velocity phase
    else if (currentTime < rampUpTime + cruiseTime) {
        float rampUpDistance = 0.5 * maxSpeed * rampUpTime;
        return rampUpDistance + (currentTime - rampUpTime) * maxSpeed;
    }

    // Deceleration phase
    else if (currentTime < rampUpTime + cruiseTime + rampDownTime) {
        float rampUpDistance = 0.5 * maxSpeed * rampUpTime;
        float cruiseDistance = cruiseTime * maxSpeed;
        float decelerationProgress = (currentTime - rampUpTime - cruiseTime) / rampDownTime;
        float decelerationDistance = quinticIntegral(decelerationProgress) * rampDownTime * maxSpeed;

        return rampUpDistance + cruiseDistance + decelerationDistance;
    }

    // Limit top
    else {
        float rampUpDistance = 0.5 * maxSpeed * rampUpTime;
        float cruiseDistance = cruiseTime * maxSpeed;
        float rampDownDistance = 0.5 * maxSpeed * rampDownTime;
        return rampUpDistance + cruiseDistance + rampDownDistance;
    }
}

#endif /* INC_QUINTICSPLIT_H_ */
