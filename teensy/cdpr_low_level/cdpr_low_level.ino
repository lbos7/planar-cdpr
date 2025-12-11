#include <Arduino.h>
#include "cdprCAN.h"
#include "cdprFlexCAN.h"
#include "cdprSerial.h"
#include "cdprConfig.h"
#include "cdpr.hpp"

// Instantiate ODrive objects
ODriveCAN odrv0 = createODriveObj(can_intf, ODRV0_NODE_ID);
ODriveCAN odrv1 = createODriveObj(can_intf, ODRV1_NODE_ID);
ODriveCAN odrv2 = createODriveObj(can_intf, ODRV2_NODE_ID);
ODriveCAN odrv3 = createODriveObj(can_intf, ODRV3_NODE_ID);
ODriveCAN* odrives[NUM_ODRIVES] = {&odrv0, &odrv1, &odrv2, &odrv3}; // Make sure all ODriveCAN instances are accounted for here

// Keep some application-specific user data for every ODrive.
ODriveUserData odrv0_user_data, odrv1_user_data, odrv2_user_data, odrv3_user_data;
ODriveUserData* dataStructs[NUM_ODRIVES] = {&odrv0_user_data, &odrv1_user_data, &odrv2_user_data, &odrv3_user_data};

// Defining dimension struct and pointer
CDPRDimensions cdprDimensions;
CDPRDimensions* dimPtr = &cdprDimensions;

// Defining control struct and pointer
CDPRControlParams controlParams;
CDPRControlParams* controlPtr = &controlParams;

// CDPR object creation
CDPR cdpr(odrives, dataStructs, dimPtr, controlPtr);

// Buffer for reading serial commands
String cmdBuffer = "";

void setup() {

    Serial.begin(115200);
    Serial.setTimeout(100);
    // Wait for up to 3 seconds for the serial port to be opened on the PC side.
    // If no PC connects, continue anyway.
    for (int i = 0; i < 300 && !Serial; ++i) {
        delay(100);
    }
    delay(200);

    // CDPR Setup
    Serial.println("Running cdpr setup");
    bool setupSuccess = cdpr.setup();
    if (!setupSuccess) {
        while (true);
    }
    cdpr.homingSequence();
    cdpr.setState(CDPRState::Homed);
    cdpr.pretensionSetup();
    cdpr.addPretension();
    cdpr.setState(CDPRState::Active);
}

void loop() {

    // Update method
    cdpr.update();

    // Reading serial commands
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            processCommand(cmdBuffer);
            cmdBuffer = "";
        } else {
            cmdBuffer += c;
        }
    }
}
