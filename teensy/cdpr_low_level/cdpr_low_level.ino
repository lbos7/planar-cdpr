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
  } else if (cmd == "RESET") {
    cdpr.homingSequence();
    cdpr.pretensionSetup();
    cdpr.addPretension();
  } else if (cmd == "TENSION") {
    cdpr.addPretension();
  } else if (cmd == "SETUPT") {
    cdpr.pretensionSetup();
  } else if (cmd == "CHECKT") {
    cdpr.checkTorques();
  } else if (cmd == "CHECKL") {
    cdpr.checkMotorPos();
    cdpr.checkLengths();
  } else if (cmd.startsWith("MOVE")) {
    // float x, y;
    // int firstSpace = cmd.indexOf(' ');
    // int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    // if (firstSpace > 0 && secondSpace > firstSpace) {
    //   x = cmd.substring(firstSpace + 1, secondSpace).toFloat();
    //   y = cmd.substring(secondSpace + 1).toFloat();
    //   moveToPosition(x, y);
    // } else {
    //   Serial.println("ERR Invalid MOVE command");
    // }
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
  cdpr.pretensionSetup();
  cdpr.addPretension();
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