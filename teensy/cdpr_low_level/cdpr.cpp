#include "cdpr.hpp"
#include "cdprFlexCAN.h"
#include <Arduino.h>


CDPR::CDPR(ODriveCAN** odrives, ODriveUserData** dataStructs, float eeSideLen) {
    this->odrives = odrives;
    this->dataStructs = dataStructs;
    this->eeSideLen = eeSideLen;
}

bool CDPR::setup() {
    bool clearSuccess = this->clearODriveErrors();

    if (!clearSuccess) {
        Serial.println("ODrive errors could not be cleared");
        return false;
    }

    this->registerCallbacks();

    if (!setupCan()) {
        Serial.println("CAN failed to initialize: reset required");
        return false;
    }

    this->checkODriveConnections();
    bool vbusCheck = this->checkODriveVBus();

    if (vbusCheck) {
        Serial.println("ODrives Connected & Powered");
    } else {
        Serial.println("ODrive Connection Error");
    }

    return vbusCheck;
}

void CDPR::homingSequence() {
    this->deactivateMotors();

    for (int i = 0; i < NUM_ODRIVES; i++) {
        this->odrives[i]->setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        this->odrives[i]->setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL,
                                            ODriveInputMode::INPUT_MODE_VEL_RAMP);

        while (!this->dataStructs[i]->received_feedback) {
            this->update();
        }
        
        Get_Encoder_Estimates_msg_t feedback = this->dataStructs[i]->last_feedback;
        this->dataStructs[i]->received_feedback = false;
        float motorSpeed = feedback.Vel_Estimate;

        uint8_t numChecks = 0;

        this->odrives[i]->setVelocity(HOMING_VELOCITY, 0.0);

        while (numChecks < HOMING_CHECK_THRESH) {
            this->update();

            if (this->dataStructs[i]->received_feedback) {
                Get_Encoder_Estimates_msg_t feedback = this->dataStructs[i]->last_feedback;
                this->dataStructs[i]->received_feedback = false;
                motorSpeed = feedback.Vel_Estimate;
                
                if (motorSpeed <= HOMING_VELOCITY_THRESH) {
                    numChecks++;
                } else {
                    numChecks = 0;
                }
            }
        }

        this->robotState.motorOffsets[i] = feedback.Pos_Estimate;
        this->odrives[i]->setVelocity(0.0, 0.0);
        // this->confirmSetControl(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_TRAP_TRAJ, i);
        this->odrives[i]->setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL,
                                            ODriveInputMode::INPUT_MODE_TRAP_TRAJ);
        this->deactivateMotors();
    }
    Serial.println("Robot Homed");
}

void CDPR::deactivateMotors() {
    for (int i = 0; i < NUM_ODRIVES; i++) {
        this->confirmSetState(ODriveAxisState::AXIS_STATE_IDLE, i);
    }
}

void CDPR::update() {
    pumpEventsWrapper(can_intf);
    if (this->completedStartup) {
        for (int i = 0; i < NUM_ODRIVES; i++) {
            if (this->dataStructs[i]->received_feedback) {
                Get_Encoder_Estimates_msg_t feedback = this->dataStructs[i]->last_feedback;
                this->dataStructs[i]->received_feedback = false;
                this->robotState.lengths[i] = this->motorPos2CableLength(feedback.Pos_Estimate, i);
            }
        }
    }
}

float CDPR::motorPos2CableLength(float motorPos, uint8_t motorID) {
    float turnsDiff = this->robotState.motorOffsets[motorID] - motorPos;
    return turnsDiff * DRUM_CIRCUMFERENCE;
}

float CDPR::cableLength2MotorPos(float cableLength, uint8_t motorID) {
    float turnsDiff = cableLength / DRUM_CIRCUMFERENCE;
    return this->robotState.motorOffsets[motorID] - turnsDiff;
}

void CDPR::registerCallbacks() {
    for (int i = 0; i < NUM_ODRIVES; i++) {
        this->odrives[i]->onFeedback(onFeedback, &this->dataStructs[i]);
        this->odrives[i]->onStatus(onHeartbeat, &this->dataStructs[i]);
        this->odrives[i]->onTorques(onTorques, &this->dataStructs[i]);
    }
}

void CDPR::confirmSetState(ODriveAxisState desiredState, uint8_t index) {
    while (this->dataStructs[index]->last_heartbeat.Axis_State != desiredState) {
        this->odrives[index]->setState(desiredState);
        this->update();
    }
}

// void CDPR::confirmSetControl(ODriveControlMode desiredControl, ODriveInputMode desiredInput, uint8_t index) {
//     while (this->dataStructs[index]->last_heartbeat.Control_Mode != desiredControl && this->dataStructs[index]->last_heartbeat.Input_Mode != desiredInput) {
//         this->odrives[index]->setControllerMode(desiredControl,
//                                                 desiredInput);
//         this->update();
//     }
// }

void CDPR::checkODriveConnections() {
    Serial.println("Checking connection to ODrives...");
    for (int i = 0; i < NUM_ODRIVES; i++) {
        Serial.printf("Waiting for ODrive %d\n", i);
        while (!this->dataStructs[i]->received_heartbeat) {
            pumpEventsWrapper(can_intf);
            delay(100);
        }
        Serial.printf("Found ODrive %d\n", i);
    }
    Serial.println("Found all ODrives");
}

bool CDPR::checkODriveVBus() {
    Serial.println("Checking bus voltage and current for ODrives...");
    for (int i = 0; i < NUM_ODRIVES; i++) {
        // request bus voltage and current (1sec timeout)
        Serial.printf("Attempting to read bus voltage and current for ODrive %d\n", i);
        Get_Bus_Voltage_Current_msg_t vbus;
        if (!this->odrives[i]->request(vbus, 1000)) {
            Serial.printf("vbus request failed for ODrive %d\n", i);
            return false;
        }
        Serial.printf("ODrive %d DC voltage [V]: %f\n", i, vbus.Bus_Voltage);
        Serial.printf("ODrive %d DC current [A]: %f\n", i, vbus.Bus_Current);
    }
    return true;
}

bool CDPR::clearODriveErrors() {
    bool totalSuccess = true;
    for (int i = 0; i < NUM_ODRIVES; i++) {
        bool success = this->odrives[i]->clearErrors();
        totalSuccess = totalSuccess && success;
    }
    return totalSuccess;
}