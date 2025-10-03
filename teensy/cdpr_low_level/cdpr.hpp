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
        void homingSequence();
        void addPretension();
        void deactivateMotors();
        void activateMotors();
        void update();
        Eigen::Vector2f solveFK(Eigen::Vector2f guess, float tol = 1e-3, uint8_t maxIter = 20);
        Eigen::Vector4f solveIK(Eigen::Vector2f eePos);
        float motorPos2CableLength(float motorPos, uint8_t motorID);
        float cableLength2MotorPos(float cableLength, uint8_t motorID);
        float torque2Tension(float torque);
        float tension2Torque(float tension);
        void changeTensionSetpoint(float tensionSetpoint);

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
        bool completedHoming = false;

        void registerCallbacks();
        void confirmSetState(ODriveAxisState desiredState, uint8_t index);
        void checkODriveConnections();
        bool checkODriveVBus();
        void clearODriveErrors();
};

#endif  // CDPR_H_