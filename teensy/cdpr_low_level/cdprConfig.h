#ifndef CDPR_CONFIG_H_
#define CDPR_CONFIG_H_

#include <math.h>

// Constants
constexpr uint32_t CAN_BAUDRATE = 250000;
constexpr uint8_t NUM_ODRIVES = 4;
constexpr float EE_SIDE_LEN = 0.0646;   // meters
constexpr float DRUM_DIAMETER = 0.05;   // meters - from CAD model
constexpr float DRUM_CIRCUMFERENCE = DRUM_DIAMETER * M_PI;  // meters
constexpr float HOMING_VELOCITY = 1.0;  // turns/s
constexpr float HOMING_VELOCITY_THRESH = 0.05;   // turns/s
constexpr uint8_t HOMING_CHECK_THRESH = 5;

// ODrive Node IDs
constexpr uint8_t ODRV0_NODE_ID = 0;
constexpr uint8_t ODRV1_NODE_ID = 1;
constexpr uint8_t ODRV2_NODE_ID = 2;
constexpr uint8_t ODRV3_NODE_ID = 3;

// Struct for cable data (lenghts, tensions, etc.) - same numbering scheme as ODrives
struct CDPRData {
    float lengths[NUM_ODRIVES] = {0.0, 0.0, 0.0, 0.0};
    float tensions[NUM_ODRIVES] = {0.0, 0.0, 0.0, 0.0};
    float motorOffsets[NUM_ODRIVES] = {0.0, 0.0, 0.0, 0.0};
};

#endif  // CDPR_CONFIG_H_