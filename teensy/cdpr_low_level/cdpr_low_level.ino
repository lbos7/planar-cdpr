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
ODriveUserData odrv0_user_data, odrv1_user_data, odrv2_user_data, odrv3_user_data;
ODriveUserData* dataStructs[NUM_ODRIVES] = {&odrv0_user_data, &odrv1_user_data, &odrv2_user_data, &odrv3_user_data};

CDPRDimensions cdprDimensions;
CDPRDimensions* dimPtr = &cdprDimensions;

CDPRControlParams controlParams;
CDPRControlParams* controlPtr = &controlParams;

// CDPR object creation
CDPR cdpr(odrives, dataStructs, dimPtr, controlPtr);

String cmdBuffer = "";

void processCommand(String cmd) {
  cmd.trim();  // remove whitespace like \r
  cmd.toUpperCase();

  if (cmd == "HOME") {
    cdpr.homingSequence();
  } else if (cmd == "DISABLE") {
    cdpr.deactivateMotors();
  } else if (cmd == "ENABLE") {
    cdpr.activateMotors();
  } else if (cmd == "DEBUG") {
    cdpr.setState(CDPRState::Debug);
  } else if (cmd == "RESET") {
    cdpr.setState(CDPRState::Startup);
    cdpr.homingSequence();
    cdpr.setState(CDPRState::Homed);
    cdpr.pretensionSetup();
    cdpr.addPretension();
    cdpr.setState(CDPRState::Active);
  } else if (cmd == "TENSION") {
    cdpr.addPretension();
  } else if (cmd == "SETUPT") {
    cdpr.pretensionSetup();
  } else if (cmd == "CHECKT") {
    cdpr.checkTorques();
  } else if (cmd == "CHECKL") {
    cdpr.checkMotorPos();
    cdpr.checkLengths();
  } else if (cmd == "CHECKP") {
    cdpr.checkEEPos();
  } else if (cmd == "CHECKS") {
    cdpr.checkState();
  } else if (cmd.startsWith("SETS ")) {
    String stateStr = cmd.substring(5);
    CDPRState newState;

    if (stateStr == "ACTIVE") {
        newState = CDPRState::Active;
    } else if (stateStr == "DEBUG") {
        newState = CDPRState::Debug;
    } else {
        Serial.println("ERR Invalid state");
        return;
    }

    cdpr.setState(newState);
    Serial.print("OK State set to ");
    Serial.println(stateStr);
  } else if (cmd.startsWith("MOVE")) {
    float x, y, speed;

    int firstSpace  = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    int thirdSpace  = cmd.indexOf(' ', secondSpace + 1);

    // Need 3 numbers (so thirdSpace == -1 only if there are no extra spaces)
    if (firstSpace > 0 && secondSpace > firstSpace) {
        // If thirdSpace == -1, there are exactly 3 arguments (x, y, speed)
        if (thirdSpace == -1) thirdSpace = cmd.length();

        x = cmd.substring(firstSpace + 1, secondSpace).toFloat();
        y = cmd.substring(secondSpace + 1, thirdSpace).toFloat();
        speed = cmd.substring(thirdSpace + 1).toFloat();

        cdpr.startTraj(Eigen::Vector2f(x, y), speed);
        Serial.print("OK Starting trajectory to (");
        Serial.print(x); Serial.print(", "); Serial.print(y);
        Serial.print(") @ "); Serial.print(speed); Serial.println(" m/s");
    } else {
        Serial.println("ERR Invalid MOVE command (expected: MOVE x y speed)");
    }
  } else {
    Serial.println("ERR Unknown command");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);
  // Wait for up to 3 seconds for the serial port to be opened on the PC side.
  // If no PC connects, continue anyway.
  for (int i = 0; i < 300 && !Serial; ++i) {
    delay(100);
  }
  delay(200);

  Serial.println("In setup()");
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
  cdpr.update();

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      processCommand(cmdBuffer);  // Step 3: Handle full command
      cmdBuffer = "";             // Step 4: Reset
    } else {
      cmdBuffer += c;             // Keep building string
    }
  }

}