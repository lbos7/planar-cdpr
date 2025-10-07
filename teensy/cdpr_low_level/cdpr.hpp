#ifndef CDPR_H_
#define CDPR_H_

#include "ODriveCAN.h"
#include "cdprCAN.h"
#include "cdprConfig.h"
#include "eigen.h"
#include <Eigen/Dense>


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
        void homingSequence();
        void pretensionSetup();
        void addPretension();
        void deactivateMotors();
        void activateMotors();
        void update();
        void setState(CDPRState state);
        CDPRState getState();
        Eigen::Vector2f solveFK(Eigen::Vector2f guess = Eigen::Vector2f::Zero(), float tol = 1e-3, uint8_t maxIter = 20);
        Eigen::Vector4f solveIK(Eigen::Vector2f eePos);
        float motorPos2CableLength(float motorPos, uint8_t motorID);
        float cableLength2MotorPos(float cableLength, uint8_t motorID);
        float torque2Tension(float torque);
        float tension2Torque(float tension);
        void changeTensionSetpoint(float tensionSetpoint);
        void startTraj(Eigen::Vector2f goal, float speed);
        void updateTraj();

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
        CDPRData robotState;
        CDPRState currentState = CDPRState::Startup;
        bool completedHoming = false;
        bool completedPretension = false;
        bool trajActive = false;
        bool hold = false;
        float trajStartTime = 0.0;
        float trajDuration = 0.0;
        float lastUpdateTime = 0.0;
        Eigen::Vector2f startPos = Eigen::Vector2f::Zero();
        Eigen::Vector2f goalPos = Eigen::Vector2f::Zero();
        Eigen::Vector2f currentPos = Eigen::Vector2f::Zero();
        Eigen::Vector2f prevPos = Eigen::Vector2f::Zero();
        Eigen::Vector2f holdPos = Eigen::Vector2f::Zero();

        void registerCallbacks();
        void confirmSetState(ODriveAxisState desiredState, uint8_t index);
        void checkODriveConnections();
        bool checkODriveVBus();
        void clearODriveErrors();
};

#endif  // CDPR_H_