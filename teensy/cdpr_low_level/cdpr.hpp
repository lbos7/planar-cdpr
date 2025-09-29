#ifndef CDPR_H_
#define CDPR_H_

#include "ODriveCAN.h"
#include "cdprCAN.h"
#include "cdprConfig.h"

class CDPR {
    public:
        CDPR(ODriveCAN** odrives, ODriveUserData** dataStructs, float eeSideLen);

        bool setup();
        void homingSequence();
        void deactivateMotors();
        void update();
        float motorPos2CableLength(float motorPos, uint8_t motorID);
        float cableLength2MotorPos(float cableLength, uint8_t motorID);

    private:
        ODriveCAN** odrives;
        ODriveUserData** dataStructs;
        float eeSideLen;
        CDPRData robotState;
        bool completedStartup = false;

        void registerCallbacks();
        void confirmSetState(ODriveAxisState desiredState, uint8_t index);
        // void confirmSetControl(ODriveControlMode desiredControl, ODriveInputMode desiredInput, uint8_t index);
        void checkODriveConnections();
        bool checkODriveVBus();
        bool clearODriveErrors();
};

#endif  // CDPR_H_