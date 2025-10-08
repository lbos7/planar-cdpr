#ifndef CDPR_CONFIG_H_
#define CDPR_CONFIG_H_

#include "eigen.h"
#include <Eigen/Dense>
#include <math.h>

// Constants
constexpr uint32_t CAN_BAUDRATE = 1000000;
constexpr uint8_t NUM_ODRIVES = 4;
constexpr float EE_SIDE_LEN = 0.0646;   // meters
constexpr float DRUM_RADIUS = 0.025;   // meters - from CAD model
constexpr float DRUM_CIRCUMFERENCE = 2*DRUM_RADIUS * M_PI;  // meters
constexpr float WORKSPACE_LEN = 0.86995;  // meters - from CAD model
constexpr float WORKSPACE_BORDER_OFFSET = 0.0254;   // meters
constexpr float TENSION_SETPOINT = 25.0; // Newtons
constexpr float HOMING_VELOCITY = 2.0;  // turns/s
constexpr float HOMING_VELOCITY_THRESH = 0.05;   // turns/s
constexpr uint8_t HOMING_CHECK_THRESH = 5;

// ODrive Node IDs
constexpr uint8_t ODRV0_NODE_ID = 0;
constexpr uint8_t ODRV1_NODE_ID = 1;
constexpr uint8_t ODRV2_NODE_ID = 2;
constexpr uint8_t ODRV3_NODE_ID = 3;

// Struct for CDPR dimensions - modify constants above if changes need to be made
struct CDPRDimensions {
    float eeSideLen = EE_SIDE_LEN;
    float drumRadius = DRUM_RADIUS;
    float drumCircumference = DRUM_CIRCUMFERENCE;
    float workspaceLen = WORKSPACE_LEN;
    float workspaceBorderOffset = WORKSPACE_BORDER_OFFSET;
    Eigen::Matrix<float, 4, 2> anchorPoints = 
        (Eigen::Matrix<float, 4, 2>() << 
            WORKSPACE_LEN/2, WORKSPACE_LEN/2,
            WORKSPACE_LEN/2, -WORKSPACE_LEN/2,
            -WORKSPACE_LEN/2, WORKSPACE_LEN/2,
            -WORKSPACE_LEN/2, -WORKSPACE_LEN/2).finished();
    Eigen::Matrix<float, 4, 2> eeOffsets = 
        (Eigen::Matrix<float, 4, 2>() << 
            EE_SIDE_LEN/2, EE_SIDE_LEN/2,
            EE_SIDE_LEN/2, -EE_SIDE_LEN/2,
            -EE_SIDE_LEN/2, EE_SIDE_LEN/2,
            -EE_SIDE_LEN/2, -EE_SIDE_LEN/2).finished();
};

// Struct for CDPR control parameters
struct CDPRControlParams {
    float tensionSetpoint = TENSION_SETPOINT;
    float homingVelocity = HOMING_VELOCITY;
    float homingVelThresh = HOMING_VELOCITY_THRESH;
    uint8_t homingCheckThresh = HOMING_CHECK_THRESH;
};

// Struct for cable data (lenghts, tensions, etc.) - same numbering scheme as ODrives
struct CDPRData {
    Eigen::Vector4f lengths = Eigen::Vector4f::Zero();
    Eigen::Vector4f tensions = Eigen::Vector4f::Zero();
    Eigen::Vector4f motorOffsets = Eigen::Vector4f::Zero();
};

enum class CDPRState {
    Startup,
    Homed,
    Active,
    Debug
};

#endif  // CDPR_CONFIG_H_