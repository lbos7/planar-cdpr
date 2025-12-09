#include "cdprSerial.h"

void processCommand(String cmd, CDPR cdpr) {
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
    } else if (cmd == "CHECKF") {
        cdpr.checkTensionsAtPos();
    } else if (cmd == "TOGGLE") {
        cdpr.toggleFF();
    } else if (cmd == "GRIDTEST") {
        cdpr.startGridTest();
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
    } else if (cmd.startsWith("LOADS")) {
        float sideLen, x, y;
    
        int firstSpace  = cmd.indexOf(' ');
        int secondSpace = cmd.indexOf(' ', firstSpace + 1);
        int thirdSpace  = cmd.indexOf(' ', secondSpace + 1);
    
        // Need 3 numbers (so thirdSpace == -1 only if there are no extra spaces)
        if (firstSpace > 0 && secondSpace > firstSpace) {
            // If thirdSpace == -1, there are exactly 3 arguments (x, y, speed)
            if (thirdSpace == -1) thirdSpace = cmd.length();
    
            sideLen = cmd.substring(firstSpace + 1, secondSpace).toFloat();
            x = cmd.substring(secondSpace + 1, thirdSpace).toFloat();
            y = cmd.substring(thirdSpace + 1).toFloat();
    
            // cdpr.startTraj(Eigen::Vector2f(x, y), speed);
            Serial.printf("Loaded Square Waypoints with side length: %.3f & centered @ x=%.3f, y=%.3f\n", sideLen, x, y);
            cdpr.loadSquareTraj(sideLen, Eigen::Vector2f(x, y));
      } else {
          Serial.println("ERR Invalid LOADS command (expected: LOADS sideLen x y)");
      }
    } else if (cmd.startsWith("LOADD")) {
        float sideLen, x, y;
  
        int firstSpace  = cmd.indexOf(' ');
        int secondSpace = cmd.indexOf(' ', firstSpace + 1);
        int thirdSpace  = cmd.indexOf(' ', secondSpace + 1);
  
        // Need 3 numbers (so thirdSpace == -1 only if there are no extra spaces)
        if (firstSpace > 0 && secondSpace > firstSpace) {
            // If thirdSpace == -1, there are exactly 3 arguments (x, y, speed)
            if (thirdSpace == -1) thirdSpace = cmd.length();
    
            sideLen = cmd.substring(firstSpace + 1, secondSpace).toFloat();
            x = cmd.substring(secondSpace + 1, thirdSpace).toFloat();
            y = cmd.substring(thirdSpace + 1).toFloat();
    
            // cdpr.startTraj(Eigen::Vector2f(x, y), speed);
            Serial.printf("Loaded Diamond Waypoints with side length: %.3f & centered @ x=%.3f, y=%.3f\n", sideLen, x, y);
            cdpr.loadDiamondTraj(sideLen, Eigen::Vector2f(x, y));
        } else {
            Serial.println("ERR Invalid LOADD command (expected: LOADD sideLen x y)");
        }
    } else if (cmd == "WAYPOINTS") {
        cdpr.activateWaypoints();
    } else if (cmd.startsWith("TWAYPOINTS")) {
        float speed;
        int firstSpace  = cmd.indexOf(' ');
    
        if (firstSpace > 0) {
            speed = cmd.substring(firstSpace + 1).toFloat();
            cdpr.activateWaypointsTraj(speed);
        } else {
            Serial.println("ERR Invalid WAYPOINTST command (expected: WAYPOINTST speed)");
        }
    } else if (cmd.startsWith("MOVE")) {
        float x, y;
    
        int firstSpace  = cmd.indexOf(' ');
        int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    
        // Need 3 numbers (so thirdSpace == -1 only if there are no extra spaces)
        if (firstSpace > 0 && secondSpace > firstSpace) {
    
            x = cmd.substring(firstSpace + 1, secondSpace).toFloat();
            y = cmd.substring(secondSpace + 1).toFloat();
    
            // cdpr.startTraj(Eigen::Vector2f(x, y), speed);
            // Serial.print("OK Changing Desired Pos to (");
            // Serial.print(x); Serial.print(", "); Serial.println(y);
            cdpr.setDesiredPos(Eigen::Vector2f(x, y));
        } else {
            Serial.println("ERR Invalid MOVE command (expected: MOVE x y)");
        }
    } else if (cmd.startsWith("TMOVE")) {
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
            // Serial.print("OK Starting trajectory to (");
            // Serial.print(x); Serial.print(", "); Serial.print(y);
            // Serial.print(") @ "); Serial.print(speed); Serial.println(" m/s");
            cdpr.generateTrajVars(Eigen::Vector2f(x, y), speed);
        } else {
            Serial.println("ERR Invalid MOVE command (expected: MOVE x y speed)");
        }
    } else if (cmd.startsWith("LOG")) {
        float x, y;
    
        int firstSpace  = cmd.indexOf(' ');
        int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    
        // Need 3 numbers (so thirdSpace == -1 only if there are no extra spaces)
        if (firstSpace > 0 && secondSpace > firstSpace) {
    
            x = cmd.substring(firstSpace + 1, secondSpace).toFloat();
            y = cmd.substring(secondSpace + 1).toFloat();
    
            // cdpr.startTraj(Eigen::Vector2f(x, y), speed);
            // Serial.print("OK Changing Desired Pos to (");
            // Serial.print(x); Serial.print(", "); Serial.println(y);
            cdpr.logPos(x, y);
        } else {
            Serial.println("ERR Invalid MOVE command (expected: MOVE x y)");
        }
    } else {
        Serial.println("ERR Unknown command");
    }
}