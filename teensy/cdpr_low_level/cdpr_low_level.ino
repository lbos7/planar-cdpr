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

// For repeated loop timing measurement
const int N_LOOPS = 150;       // number of loops to average
unsigned long loopTimeSum = 0;
int loopCount = 0;
bool measuring = true;          // true when currently measuring

elapsedMicros loopTimer;

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
  } else if (cmd == "CHECKG") {
    cdpr.checkGains();
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
  } else if (cmd.startsWith("SETG")) {
    float Kp, Kd, Ki;

    int firstSpace  = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    int thirdSpace  = cmd.indexOf(' ', secondSpace + 1);

    // Need 3 numbers (so thirdSpace == -1 only if there are no extra spaces)
    if (firstSpace > 0 && secondSpace > firstSpace) {
        // If thirdSpace == -1, there are exactly 3 arguments (x, y, speed)
        if (thirdSpace == -1) thirdSpace = cmd.length();

        Kp = cmd.substring(firstSpace + 1, secondSpace).toFloat();
        Kd = cmd.substring(secondSpace + 1, thirdSpace).toFloat();
        Ki = cmd.substring(thirdSpace + 1).toFloat();

            Serial.print("OK Gains set: Kp = ");
            Serial.print(Kp);
            Serial.print(", Kd = ");
            Serial.print(Kd);
            Serial.print(", Ki = ");
            Serial.println(Ki);
        cdpr.setGains(Kp, Kd, Ki);
    } else {
        Serial.println("ERR Invalid SETG command (expected: SETG Kp Kd Ki)");
    }
  } else if (cmd.startsWith("SETGT")) {
    float Kp, Kd;

    int firstSpace  = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);

    // Need exactly 2 numbers
    if (firstSpace > 0 && secondSpace > firstSpace) {
        Kp = cmd.substring(firstSpace + 1, secondSpace).toFloat();
        Kd = cmd.substring(secondSpace + 1).toFloat();

        // Example: set gains in your CDPR control object
        cdpr.setGains(Kp, Kd, 0.0f);

        Serial.print("OK Gains set: Kp = ");
        Serial.print(Kp);
        Serial.print(", Kd = ");
        Serial.println(Kd);
    } else {
        Serial.println("ERR Invalid SETG command (expected: SETGT Kp Kd)");
    }
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

        // cdpr.startTraj(Eigen::Vector2f(x, y), speed);
        Serial.print("OK Starting trajectory to (");
        Serial.print(x); Serial.print(", "); Serial.print(y);
        Serial.print(") @ "); Serial.print(speed); Serial.println(" m/s");
        cdpr.setDesiredPos(Eigen::Vector2f(x, y));
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
  cdpr.setState(CDPRState::Debug);
}

void loop() {
  loopTimer = 0;  // reset timer at start of each loop

  cdpr.update();

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      processCommand(cmdBuffer);
      cmdBuffer = "";
    } else {
      cmdBuffer += c;
    }
  }

  // // --- Loop timing measurement ---
  // if (measuring) {
  //   loopTimeSum += loopTimer;
  //   loopCount++;

  //   if (loopCount >= N_LOOPS) {
  //     float avgLoopTime = (float)loopTimeSum / loopCount;
  //     Serial.print("Average loop time over ");
  //     Serial.print(N_LOOPS);
  //     Serial.print(" loops: ");
  //     Serial.print(avgLoopTime);
  //     Serial.println(" us");

  //     // Reset counters to measure again on next N_LOOPS
  //     loopTimeSum = 0;
  //     loopCount = 0;
  //     // Keep measuring for multiple sets
  //     // Optionally you can insert a short delay or a trigger condition here
  //   }
  // }
}
