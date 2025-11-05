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
        CDPR(ODriveCAN** odrives,
             ODriveUserData** dataStructs,
             CDPRDimensions* dimPtr,
             CDPRControlParams* controlPtr);

        bool setup();
        void checkTorques();
        void checkMotorPos();
        void checkLengths();
        void checkEEPos();
        void checkState();
        void checkGains();
        void checkTensionsAtPos();
        void homingSequence();
        void pretensionSetup();
        void addPretension();
        void deactivateMotors();
        void activateMotors();
        void update();
        void setState(CDPRState state);
        CDPRState getState();
        void setGains(float Kp, float Kd, float Ki);
        Eigen::Vector2f solveFK(Eigen::Vector2f guess = Eigen::Vector2f::Zero(), float tol = 1e-3, uint8_t maxIter = 20);
        Eigen::Vector4f solveIK(Eigen::Vector2f eePos);
        float motorPos2CableLength(float motorPos, uint8_t motorID);
        float cableLength2MotorPos(float cableLength, uint8_t motorID);
        float torque2Tension(float torque);
        float tension2Torque(float tension);
        void changeTensionSetpoint(float tensionSetpoint);
        void setDesiredPos(Eigen::Vector2f pos);
        void applyController(float dt);
        void applyFFController(float dt);
        Eigen::Vector4f computeTensionsFromForce(Eigen::Vector2f &force);
        Eigen::Vector4f computeTensionsFF(Eigen::Vector2f &force);
        Eigen::Vector2f computeForceFromTensions(Eigen::Vector4f &tensions);
        Eigen::Matrix<float, 4, 2> computeCableUnitVecs();
        void loadSquareTraj(float sideLen, Eigen::Vector2f center = Eigen::Vector2f::Zero());
        void loadDiamondTraj(float sideLen, Eigen::Vector2f center = Eigen::Vector2f::Zero());
        void activateWaypoints();
        void activateWaypointsTraj(float speed);
        void generateTrajVars(Eigen::Vector2f goalPos, float speed);
        void manageWaypoints();
        void updateTraj(float dt);
        void startGridTest();
        void updateGridTest();
        Eigen::Matrix<float, 10, 1> computeFFBasis();
        void toggleFF();

    private:
        ODriveCAN** odrives;
        ODriveUserData** dataStructs;
        float eeSideLen;
        float drumRadius;
        float drumCircumference;
        float workspaceLen;
        float workspaceBorderOffset;
        Eigen::Matrix<float, 4, 2> anchorPoints;
        Eigen::Matrix<float, 4, 2> eeOffsets;
        float tensionSetpoint;
        float homingVelocity;
        float homingVelThresh;
        uint8_t homingCheckThresh;
        float Kp;
        float Kd;
        float Ki;
        float tau;
        float holdThresh;
        float maxTension;
        float minTension;
        Eigen::Matrix<float, 4, 10> ffCoeffs;
        Eigen::Vector2f intError = Eigen::Vector2f::Zero();
        CDPRData robotData;
        CDPRState robotState = CDPRState::Startup;
        bool completedHoming = false;
        bool completedPretension = false;
        bool trajActive = false;
        float trajSpeed = 0.0;
        bool hold = false;
        float s = 0.0;
        float segmentLen = 0.0;
        Eigen::Vector2f segmentDir = Eigen::Vector2f::Zero();
        Eigen::Vector2f startPos = Eigen::Vector2f::Zero();
        Eigen::Vector2f goalPos = Eigen::Vector2f::Zero();
        Eigen::Vector2f intermediatePos = Eigen::Vector2f::Zero();
        Eigen::Vector2f eePos = Eigen::Vector2f::Zero();
        Eigen::Vector2f desiredPos = Eigen::Vector2f::Zero();
        Eigen::Vector2f eeVel = Eigen::Vector2f::Zero();
        Eigen::Vector2f desiredVel = Eigen::Vector2f::Zero();
        Eigen::Vector2f prevPos = Eigen::Vector2f::Zero();
        Eigen::Vector2f holdPos = Eigen::Vector2f::Zero();
        Eigen::Vector2f error = Eigen::Vector2f::Zero();
        Eigen::Vector2f dedt = Eigen::Vector2f::Zero();
        Eigen::Vector2f prevError = Eigen::Vector2f::Zero();
        float prevUpdateTime = 0.0;
        std::vector<Eigen::Vector2f> waypoints;
        float waypointDistThresh = 0.01;
        uint8_t currentWaypointInd = 0;
        float waypointSpeed = 0.0;
        bool completedWaypoints = false;
        bool useWaypointsTraj = false;
        float gridTestSpeed = 0.25;
        int gridIndX = 0;
        int gridIndY = 0;
        // float gridCheckpoints[12] = {
        //     -0.393475, -0.321934, -0.250393, -0.178852,
        //     -0.107311, -0.035770, 0.035770, 0.107311,
        //     0.178852, 0.250393, 0.321934, 0.393475
        // };
        float gridCheckpoints[11] = {
            -0.393475, -0.314543, -0.235611, -0.156679,
            -0.077747,  0.001185,  0.080117,  0.159049,
            0.237981,  0.316913,  0.395845
        };
        bool firstGridPoint = true;
        Eigen::Vector2f lastLoggedPos = Eigen::Vector2f::Zero();
        bool useFF = false;

        void registerCallbacks();
        void confirmSetState(ODriveAxisState desiredState, uint8_t index);
        void checkODriveConnections();
        bool checkODriveVBus();
        void clearODriveErrors();
};

#endif  // CDPR_H_