#ifndef CDPR_CONFIG_H_
#define CDPR_CONFIG_H_

#include "eigen.h"
#include <Eigen/Dense>
#include <math.h>

// Constants
constexpr uint32_t CAN_BAUDRATE = 1000000; ///< CAN bus speed (bps)
constexpr uint8_t NUM_ODRIVES = 4; ///< Total number of ODrive motor controllers
constexpr float EE_SIDE_LEN = 0.0426; ///< EE side length (meters)
constexpr float DRUM_RADIUS = 0.025; ///< Drum radius (meters)
constexpr float DRUM_CIRCUMFERENCE = 2*DRUM_RADIUS * M_PI; ///< Drum circumference (meters)
constexpr float WORKSPACE_LEN = 0.86995; ///< Workspace along X/Y (meters)
constexpr float TENSION_SETPOINT = 30.0; ///< Target cable tension (N)
constexpr float HOMING_VELOCITY = 2.0; ///< Motor homing speed (turns/s)
constexpr float HOMING_VELOCITY_THRESH = 0.05; ///< Velocity to detect homing (turns/s)
constexpr uint8_t HOMING_CHECK_THRESH = 5; ///< Number of consecutive checks needed to stop homing
constexpr float KP = 1500.0; ///< Proportional gain
constexpr float KD = 50.0; ///< Derivative gain
constexpr float KI = 0.0; ///< Integral gain
constexpr float TAU = 0.015; ///< Filter time constant
constexpr float MAX_TENSION = 55.0; ///< Max allowable tension (N)
constexpr float MIN_TENSION = 8.0; ///< Min allowable tension (N)
extern Eigen::Matrix<float, 4, 10> FF_COEFFS; ///< 4x10 feedforward model coefficient

// ODrive Node IDs
constexpr uint8_t ODRV0_NODE_ID = 0; ///< ODrive 0 CAN ID
constexpr uint8_t ODRV1_NODE_ID = 1; ///< ODrive 1 CAN ID
constexpr uint8_t ODRV2_NODE_ID = 2; ///< ODrive 2 CAN ID
constexpr uint8_t ODRV3_NODE_ID = 3; ///< ODrive 3 CAN ID

/** 
 * @brief Struct of physical dimensions and geometry of the CDPR.
 */
struct CDPRDimensions {
    float eeSideLen = EE_SIDE_LEN;               ///< End-effector side length
    float drumRadius = DRUM_RADIUS;             ///< Drum radius
    float drumCircumference = DRUM_CIRCUMFERENCE; ///< Drum circumference
    float workspaceLen = WORKSPACE_LEN;         ///< Workspace size

    Eigen::Matrix<float, 4, 2> anchorPoints =   ///< Fixed anchor points in base frame
        (Eigen::Matrix<float, 4, 2>() << 
            WORKSPACE_LEN/2, WORKSPACE_LEN/2,
            WORKSPACE_LEN/2, -WORKSPACE_LEN/2,
            -WORKSPACE_LEN/2, WORKSPACE_LEN/2,
            -WORKSPACE_LEN/2, -WORKSPACE_LEN/2).finished();

    Eigen::Matrix<float, 4, 2> eeOffsets =      ///< Cable offsets on the EE
        (Eigen::Matrix<float, 4, 2>() << 
            EE_SIDE_LEN/2, EE_SIDE_LEN/2,
            EE_SIDE_LEN/2, -EE_SIDE_LEN/2,
            -EE_SIDE_LEN/2, EE_SIDE_LEN/2,
            -EE_SIDE_LEN/2, -EE_SIDE_LEN/2).finished();
};

/** 
 * @brief Struct of control parameters for the CDPR object.
 */
struct CDPRControlParams {
    float tensionSetpoint = TENSION_SETPOINT;   ///< Desired cable tension
    float homingVelocity = HOMING_VELOCITY;    ///< Homing speed
    float homingVelThresh = HOMING_VELOCITY_THRESH; ///< Threshold to detect homing
    uint8_t homingCheckThresh = HOMING_CHECK_THRESH; ///< Number of checks
    float Kp = KP;                              ///< Proportional gain
    float Kd = KD;                              ///< Derivative gain
    float Ki = KI;                              ///< Integral gain
    float tau = TAU;                            ///< Filter time constant
    float maxTension = MAX_TENSION;            ///< Max allowable tension
    float minTension = MIN_TENSION;            ///< Min allowable tension
    Eigen::Matrix<float, 4, 10> ffCoeffs = FF_COEFFS; ///< Feedforward coefficients
};

/** 
 * @brief Struct for cable measurement data.
 */
struct CDPRData {
    Eigen::Vector4f lengths = Eigen::Vector4f::Zero();       ///< Cable lengths
    Eigen::Vector4f tensions = Eigen::Vector4f::Zero();      ///< Cable tensions
    Eigen::Vector4f motorOffsets = Eigen::Vector4f::Zero();  ///< Motor offsets
};

/** 
 * @brief State of the CDPR object.
 */
enum class CDPRState {
    Startup,   ///< System starting up
    Homed,     ///< Homing completed
    Active,    ///< Actively controlling EE
    Waypoint,  ///< Moving along waypoints
    GridTest,  ///< Running grid test for feedforward model
    Debug      ///< Debug mode
};

#endif  // CDPR_CONFIG_H_