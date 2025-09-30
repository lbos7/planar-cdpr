#include <Arduino.h>
#include "cdprCAN.h"
#include "cdprFlexCAN.h"
#include "cdprConfig.h"
#include "cdpr.hpp"

// Instantiate ODrive objects
ODriveCAN odrv0 = createODriveObj(can_intf, ODRV0_NODE_ID);
ODriveCAN odrv1 = createODriveObj(can_intf, ODRV1_NODE_ID);
ODriveCAN odrv2 = createODriveObj(can_intf, ODRV2_NODE_ID);
ODriveCAN odrv3 = createODriveObj(can_intf, ODRV3_NODE_ID);
ODriveCAN* odrives[NUM_ODRIVES] = {&odrv0, &odrv1, &odrv2, &odrv3}; // Make sure all ODriveCAN instances are accounted for here

// Keep some application-specific user data for every ODrive.
ODriveUserData odrv0_user_data;
ODriveUserData odrv1_user_data;
ODriveUserData odrv2_user_data;
ODriveUserData odrv3_user_data;
ODriveUserData* dataStructs[NUM_ODRIVES] = {&odrv0_user_data, &odrv1_user_data, &odrv2_user_data, &odrv3_user_data};

CDPRDimensions cdprDimensions;
CDPRDimensions* dimPtr = &cdprDimensions;

CDPRControlParams controlParams;
CDPRControlParams* controlPtr = &controlParams;

// CDPR object creation
CDPR cdpr(odrives, dataStructs, dimPtr, controlPtr);

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);
  bool setupSuccess = cdpr.setup();
  if (!setupSuccess) {
    while (true);
  }
  cdpr.homingSequence();
}

void loop() {
  cdpr.update();
}