#ifndef CDPR_H_
#define CDPR_H_

#include "ODriveCAN.h"
#include "cdprCAN.h"
#include "cdprConfig.h"
#include "eigen.h"
#include <Eigen/Dense>
#include <vector>


class CDPR {
    
    public:

        /**
         * @brief Construct a new CDPR object.
         * 
         * @param odrives The ODrives that will be used to operate the system.
         * @param dataStructs The feedback structs for each ODrive.
         * @param dimPtr The pointer to a struct of relevant system dimensions.
         * @param controlPtr The pointer to a struct fo relevant system control paramters.
         */
        CDPR(ODriveCAN** odrives,
             ODriveUserData** dataStructs,
             CDPRDimensions* dimPtr,
             CDPRControlParams* controlPtr);

        /**
         * @brief Checks to make sure CAN bus is working and ODrives are connected.
         * 
         * @return true 
         * @return false 
         */
        bool setup();

        /**
         * @brief Prints the target and estimated torque for each Motor.
         */
        void checkTorques();

        /**
         * @brief Prints the position for each Motor.
         */
        void checkMotorPos();

        /**
         * @brief Prints the length of each cable.
         */
        void checkLengths();

        /**
         * @brief Prints the current end-effector position.
         */
        void checkEEPos();

        /**
         * @brief Prints the current state of the CDPR object.
         */
        void checkState();

        /**
         * @brief Prints the current controller gains.
         */
        void checkGains();

        /**
         * @brief Prints the current end-effector position and cable tensions over Serial.
         */
        void checkTensionsAtPos();

        /**
         * @brief Logs the current end-effector position and a reference position over Serial.
         * 
         * @param x Reference X position to log.
         * @param y Reference Y position to log.
         */
        void logPos(float x, float y);

        /**
         * @brief Turns each motor until the end-effector reaches the corresponding pulley and logs motor position.
         */
        void homingSequence();

        /**
         * @brief Sets up the system for pretensioning by adjusting cable lengths.
         */
        void pretensionSetup();

        /**
         * @brief Adds a pretension force to each of the cables.
         */
        void addPretension();

        /**
         * @brief Sets all ODrives to idle.
         */
        void deactivateMotors();

        /**
         * @brief Sets all ODrives to closed-loop position control mode.
         */
        void activateMotors();

        /**
         * @brief Updates important variables/attributes based on the CDPR object's state.
         */
        void update();

        /**
         * @brief Changes the state enum of the CDPR object.
         * 
         * @param state The desired state for the CDPR object.
         */
        void setState(CDPRState state);

        /**
         * @brief Returns the current state of the CDPR object.
         * 
         * @return CDPRState The current state of the CDPR object.
         */
        CDPRState getState();

        /**
         * @brief Sets the controller gains of the CDPR object.
         * 
         * @param Kp The proportional gain.
         * @param Kd The derivative gain.
         * @param Ki The integral gain.
         */
        void setGains(float Kp, float Kd, float Ki);

        /**
         * @brief Finds the end-effector's x,y position from motor positions/cable lengths through forward kinematics.
         * 
         * @param guess The initial end-effector position guess.
         * @param tol The distance tolerance for estimating end-effector position.
         * @param maxIter The max number of iterations before breaking the loop.
         * @return Eigen::Vector2f The end-effector's x,y position.
         */
        Eigen::Vector2f solveFK(Eigen::Vector2f guess = Eigen::Vector2f::Zero(), float tol = 1e-3, uint8_t maxIter = 20);

        /**
         * @brief Finds cable lengths from end-effector x,y position through inverse kinematics.
         * 
         * @param eePos The end-effector x,y position.
         * @return Eigen::Vector4f The 4 cable lengths.
         */
        Eigen::Vector4f solveIK(Eigen::Vector2f eePos);

        /**
         * @brief Converts from motor position to cable length.
         * 
         * @param motorPos The motor position to convert form (turns).
         * @param motorID The id # corresponding to which motor the conversion is for.
         * @return float The cable length corresponding to the motor position (m).
         */
        float motorPos2CableLength(float motorPos, uint8_t motorID);

        /**
         * @brief Converts from cable length to motor position.
         * 
         * @param cableLength The cable length to convert from (m).
         * @param motorID The id # corresponding to which motor the conversion is for.
         * @return float The motor position corresponding to the cable length (turns).
         */
        float cableLength2MotorPos(float cableLength, uint8_t motorID);

        /**
         * @brief Converts from motor torque to cable tension.
         * 
         * @param torque The motor torque to convert from (Nm).
         * @return float The cable tension corresponding to the motor torque (N).
         */
        float torque2Tension(float torque);

        /**
         * @brief Converts from cable tension to motor torque.
         * 
         * @param tension The cable tension to convert from (N).
         * @return float The motor torque corresponding to the motor torque (Nm).
         */
        float tension2Torque(float tension);

        /**
         * @brief Applies a PID + Feedforward controller based on position and velocity error.
         * 
         * @param dt The difference in time since the last control loop execution.
         */
        void applyController(float dt);

        /**
         * @brief Computes 4 cable tensions from 2D force.
         * 
         * @param force The force to decompose into tensions (N).
         * @return Eigen::Vector4f The 4 cable tensions corresponding to the input force (N).
         */
        Eigen::Vector4f computeControllerTensions(Eigen::Vector2f &force);

        /**
         * @brief Computes unit vectors from end-effector to anchor points for each cable.
         * 
         * @return Eigen::Matrix<float, 4, 2> The unit vectors for each cable (1 row per cable).
         */
        Eigen::Matrix<float, 4, 2> computeCableUnitVecs();

        /**
         * @brief Computes a basis vector from the desired end-effector position for use in feedforward force estimation.
         */
        Eigen::Matrix<float, 10, 1> computeFFBasis();

        /**
         * @brief Loads a waypoints arranged in a square for the end-effector to move between.
         * 
         * @param sideLen The spacing between waypoints (m).
         * @param center The x,y centerpoint between all of the waypoints (m).
         */
        void loadSquareWaypoints(float sideLen, Eigen::Vector2f center = Eigen::Vector2f::Zero());

        /**
         * @brief Loads a waypoints arranged in a diamond for the end-effector to move between.
         * 
         * @param sideLen The spacing between waypoints (m).
         * @param center The x,y centerpoint between all of the waypoints (m).
         */
        void loadDiamondWaypoints(float sideLen, Eigen::Vector2f center = Eigen::Vector2f::Zero());

        /**
         * @brief Activates waypoint-following mode using the trajectory-following controller.
         * 
         * @param speed The desired end-effector speed (m/s).
         */
        void activateWaypointsTraj(float speed = 1.0);

        /**
         * @brief Generates variables necessary for the end-effector to follow a trajectory.
         * 
         * @param goalPos The goal x,y position for the end-effector (m).
         * @param speed The desired speed for the end-effector (m/s).
         */
        void generateTrajVars(Eigen::Vector2f goalPos, float speed = 1.0);

        /**
         * @brief Handles waypoint-following mode operation and adjusts the current waypoint.
         */
        void manageWaypoints();

        /**
         * @brief Updates desired end-effector position and velocity based on timestep, desired speed, and segment length.
         * 
         * @param dt The difference in time since the last control loop execution.
         */
        void updateTraj(float dt);

        // ==========================================
        // ===== FEEDFORWARD MODEL GRID TESTING =====
        // ==========================================
        // NOTE:
        // These methods are only used for collecting new data to
        // retrain or refine the feedforward model. They should not
        // be modified or called during normal operation.

        /**
         * @brief Initializes the grid-testing routine by resetting indices, flags, and transitioning the robot into GridTest mode.
         */
        void startGridTest();

        /**
         * @brief Executes one step of the grid test, advancing through grid checkpoints, triggering tension logging, and generating trajectories to each point.
         */
        void updateGridTest();

    private:

        // Arrays of ODrive objects and struct pointers
        ODriveCAN** odrives;               ///< Array of pointers to ODriveCAN objects controlling each motor
        ODriveUserData** dataStructs;      ///< Array of pointers to feedback structs for each ODrive

        // Geometry / workspace parameters
        float eeSideLen;                   ///< End-effector side length (meters)
        float drumRadius;                  ///< Radius of the motor drum (meters)
        float drumCircumference;           ///< Circumference of the drum (meters)
        float workspaceLen;                ///< Length of the workspace along X or Y (meters)
        Eigen::Matrix<float, 4, 2> anchorPoints; ///< Fixed cable anchor points in base frame
        Eigen::Matrix<float, 4, 2> eeOffsets;    ///< Offsets of cable attachment points on the end-effector

        // Tension and homing
        float tensionSetpoint;             ///< Desired cable tension (Newtons)
        float homingVelocity;              ///< Velocity used during homing (m/s)
        float homingVelThresh;             ///< Velocity threshold to detect homing completion
        uint8_t homingCheckThresh;         ///< Number of checks required for homing validation

        // Controller gains
        float Kp;                          ///< Proportional gain
        float Kd;                          ///< Derivative gain
        float Ki;                          ///< Integral gain
        float tau;                         ///< Low-pass filter time constant for derivative term
        float holdThresh;                  ///< Threshold for hold mode
        float maxTension;                  ///< Maximum allowable cable tension (N)
        float minTension;                  ///< Minimum allowable cable tension (N)

        // Feedforward / model parameters
        Eigen::Matrix<float, 4, 10> ffCoeffs; ///< Feedforward coefficient matrix
        Eigen::Vector2f intError = Eigen::Vector2f::Zero(); ///< Integral error for controller

        // Robot state
        CDPRData robotData;                ///< Struct storing current robot data
        CDPRState robotState = CDPRState::Startup; ///< Current state of the robot
        bool completedHoming = false;      ///< True if homing sequence completed
        bool completedPretension = false;  ///< True if pretension applied
        bool trajActive = false;           ///< True if trajectory is active
        float trajSpeed = 0.0;             ///< Speed of current trajectory

        // Trajectory variables
        float s = 0.0;                     ///< Progress along current trajectory segment
        float segmentLen = 0.0;            ///< Length of current segment
        Eigen::Vector2f segmentDir = Eigen::Vector2f::Zero(); ///< Unit vector along segment
        Eigen::Vector2f startPos = Eigen::Vector2f::Zero();   ///< Start position of current segment
        Eigen::Vector2f goalPos = Eigen::Vector2f::Zero();    ///< Goal position of current segment
        Eigen::Vector2f intermediatePos = Eigen::Vector2f::Zero(); ///< Temporary position along trajectory
        Eigen::Vector2f eePos = Eigen::Vector2f::Zero();      ///< Current end-effector position
        Eigen::Vector2f desiredPos = Eigen::Vector2f::Zero(); ///< Desired end-effector position
        Eigen::Vector2f eeVel = Eigen::Vector2f::Zero();      ///< Current end-effector velocity
        Eigen::Vector2f desiredVel = Eigen::Vector2f::Zero(); ///< Desired end-effector velocity
        Eigen::Vector2f prevPos = Eigen::Vector2f::Zero();    ///< End-effector position at previous timestep
        Eigen::Vector2f holdPos = Eigen::Vector2f::Zero();    ///< Position to hold when hold mode is active
        Eigen::Vector2f error = Eigen::Vector2f::Zero();      ///< Position error (desired - actual)
        Eigen::Vector2f dedt = Eigen::Vector2f::Zero();       ///< Derivative of error
        Eigen::Vector2f prevError = Eigen::Vector2f::Zero();  ///< Error from previous timestep
        float prevUpdateTime = 0.0;                           ///< Timestamp of previous update

        // Waypoints handling
        std::vector<Eigen::Vector2f> waypoints; ///< List of waypoints for path following
        float waypointDistThresh = 0.015;       ///< Distance threshold to consider waypoint reached
        uint8_t currentWaypointInd = 0;         ///< Index of the current target waypoint
        float waypointSpeed = 0.0;              ///< Speed to move along waypoints
        bool completedWaypoints = false;        ///< True if all waypoints are completed

        // Grid test parameters
        float gridTestSpeed = 0.25;             ///< Speed for grid test motion
        int gridIndX = 0;                       ///< Current X index in grid test
        int gridIndY = 0;                       ///< Current Y index in grid test
        // float gridCheckpoints[12] = {           ///< Predefined 12x12 grid checkpoints (meters)
        //     -0.393475, -0.321934, -0.250393, -0.178852,
        //     -0.107311, -0.035770, 0.035770, 0.107311,
        //     0.178852, 0.250393, 0.321934, 0.393475
        // };
        float gridCheckpoints[11] = {           ///< Predefined 11x11 grid checkpoints (meters)
            -0.393475, -0.314543, -0.235611, -0.156679,
            -0.077747,  0.001185,  0.080117,  0.159049,
            0.237981,  0.316913,  0.395845
        };
        bool firstGridPoint = true;             ///< True if first grid point has not been reached
        Eigen::Vector2f lastLoggedPos = Eigen::Vector2f::Zero(); ///< Last position logged


        /**
         * @brief Registers callback functions to log data from the ODrives.
         */
        void registerCallbacks();

        /**
         * @brief Changes the ODrive state and confirms that the state has been set correctly. 
         * 
         * @param desiredState The desired ODriveAxisState enum.
         * @param index The ODrive index number.
         */
        void confirmSetState(ODriveAxisState desiredState, uint8_t index);

        /**
         * @brief Checks to make sure that all ODrives are connected.
         */
        void checkODriveConnections();

        /**
         * @brief Checks to make sure the ODrives' bus voltage and current can be read.
         * 
         * @return true The bus voltage and current can be read for all ODrives.
         * @return false The bus voltage and current could not be read for all ODrives.
         */
        bool checkODriveVBus();

        /**
         * @brief Clears any errors for all ODrives.
         */
        void clearODriveErrors();
};

#endif  // CDPR_H_