#include "cdpr.hpp"
#include "cdprFlexCAN.h"
#include "eigen.h"
#include <Eigen/Dense>
#include <vector>
#include <Arduino.h>
#include <math.h>


CDPR::CDPR(ODriveCAN** odrives,
           ODriveUserData** dataStructs,
           CDPRDimensions* dimPtr,
           CDPRControlParams* controlPtr) {

    this->odrives = odrives;
    this->dataStructs = dataStructs;
    this->eeSideLen = dimPtr->eeSideLen;
    this->drumRadius = dimPtr->drumRadius;
    this->drumCircumference = dimPtr->drumCircumference;
    this->workspaceLen = dimPtr->workspaceLen;
    this->workspaceBorderOffset = dimPtr->workspaceBorderOffset;
    this->anchorPoints = dimPtr->anchorPoints;
    this->eeOffsets = dimPtr->eeOffsets;
    this->tensionSetpoint = controlPtr->tensionSetpoint;
    this->homingVelocity = controlPtr->homingVelocity;
    this->homingVelThresh = controlPtr->homingVelThresh;
    this->homingCheckThresh = controlPtr->homingCheckThresh;
    this->KpHold = controlPtr->KpHold;
    this->KdHold = controlPtr->KdHold;
    this->KiHold = controlPtr->KiHold;
    this->KpTraj = controlPtr->KpTraj;
    this->KdTraj = controlPtr->KdTraj;
    this->tau = controlPtr->tau;
    this->holdThresh = controlPtr->holdThesh;
    this->maxTension = controlPtr->maxTension;
    this->minTension = controlPtr->minTension;
    this->robotData = CDPRData();
    this->waypoints.reserve(50);
}

bool CDPR::setup() {

    this->registerCallbacks();

    if (!setupCan()) {
        Serial.println("CAN failed to initialize: reset required");
        return false;
    }

    this->checkODriveConnections();
    bool vbusCheck = this->checkODriveVBus();

    if (vbusCheck) {
        Serial.println("ODrives Connected & Powered");
        this->clearODriveErrors();
    } else {
        Serial.println("ODrive Connection Error");
    }

    return vbusCheck;
}

void CDPR::checkTorques() {
    for (int i = 0; i < NUM_ODRIVES; i++) {
        while (!this->dataStructs[i]->received_torque);
        Get_Torques_msg_t torque = this->dataStructs[i]->last_torque;
        this->dataStructs[i]->received_torque = false;
        Serial.printf("Motor %d target torque:\t%f\n", i, torque.Torque_Target);
        Serial.printf("Motor %d estimated torque:\t%f\n", i, torque.Torque_Estimate);
    }
}

void CDPR::checkMotorPos() {
    for (int i = 0; i < NUM_ODRIVES; i++) {
        while (!this->dataStructs[i]->received_feedback);
        Get_Encoder_Estimates_msg_t feedback = this->dataStructs[i]->last_feedback;
        this->dataStructs[i]->received_feedback = false;
        Serial.printf("Motor %d Pos Estimate:\t%f\n", i, feedback.Pos_Estimate);
        Serial.printf("Motor %d Offset Pos:\t%f\n", i, this->robotData.motorOffsets(i));
    }
}

void CDPR::checkLengths() {
    for (int i = 0; i < NUM_ODRIVES; i++) {
        float len = this->robotData.lengths(i);
        Serial.printf("Cable %d length:\t%f\n", i, len);
    }
}

void CDPR::checkEEPos() {
    Serial.printf("Current EE Pos: [%.5f, %.5f]\n", this->eePos(0), this->eePos(1));
}

void CDPR::checkState() {
    // Print robot state
    const char* stateStr = "Unknown";

    switch (this->robotState) {  // assuming `this->state` is of type CDPRState
        case CDPRState::Startup: stateStr = "Startup"; break;
        case CDPRState::Hold:  stateStr = "Hold";  break;
        case CDPRState::Debug:   stateStr = "Debug";   break;
        case CDPRState::Homed:   stateStr = "Homed";   break;
        case CDPRState::Waypoint:   stateStr = "Waypoint";   break;
        case CDPRState::Trajectory:   stateStr = "Trajectory";   break;
    }

    Serial.print("Current State: ");
    Serial.println(stateStr);
}

void CDPR::checkGains() {

    Serial.printf("Kp: %0.3f\tKd: %0.3f\tKi: %0.3f\n", this->KpHold, this->KdHold, this->KiHold);
}

void CDPR::homingSequence() {

    Serial.println("Homing Robot");

    this->deactivateMotors();
    for (int i = 0; i < NUM_ODRIVES; i++) {
        this->confirmSetState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL, i);
        this->odrives[i]->setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL,
                                            ODriveInputMode::INPUT_MODE_VEL_RAMP);

        while (!this->dataStructs[i]->received_feedback) {
            this->update();
        }
        
        Get_Encoder_Estimates_msg_t feedback = this->dataStructs[i]->last_feedback;
        this->dataStructs[i]->received_feedback = false;
        float motorSpeed = feedback.Vel_Estimate;
        float initialPos = feedback.Pos_Estimate;

        uint8_t numChecks = 0;

        this->odrives[i]->setVelocity(this->homingVelocity, 0.0);

        // Wait until motor reaches ~90% of homing velocity and has moved at least a 1/4 turn
        while (true) {
            this->update();

            if (this->dataStructs[i]->received_feedback) {
                feedback = this->dataStructs[i]->last_feedback;
                this->dataStructs[i]->received_feedback = false;

                if (fabs(feedback.Vel_Estimate) >= this->homingVelocity * 0.9 && fabs(feedback.Pos_Estimate - initialPos) >= 0.25) {
                    break; // motor is moving enough to start homing check
                }
            }
        }
        

        while (numChecks < this->homingCheckThresh) {
            this->update();

            if (this->dataStructs[i]->received_feedback) {
                feedback = this->dataStructs[i]->last_feedback;
                this->dataStructs[i]->received_feedback = false;
                motorSpeed = feedback.Vel_Estimate;
                
                if (motorSpeed <= this->homingVelThresh) {
                    numChecks++;
                } else {
                    numChecks = 0;
                }
            }
        }

        while (!this->dataStructs[i]->received_feedback) {
            this->update();
        }
        feedback = this->dataStructs[i]->last_feedback;
        this->dataStructs[i]->received_feedback = false;
        this->odrives[i]->setVelocity(0.0, 0.0);
        this->robotData.motorOffsets(i) = feedback.Pos_Estimate;
        this->deactivateMotors();
    }
    this->completedHoming = true;
    Serial.println("Robot Homed");
}

void CDPR::pretensionSetup() {
    Serial.println("Preparing for pretension");
    Eigen::Vector4f goalLengths = this->solveIK(Eigen::Vector2f(0.0f, 0.0f));
    for (int i = 0; i < NUM_ODRIVES; i++) {
        Serial.printf("Cable %d goal length:\t%f\n", i, goalLengths(i));
        this->confirmSetState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL, i);
        this->odrives[i]->setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL,
                                            ODriveInputMode::INPUT_MODE_VEL_RAMP);
        this->odrives[i]->setVelocity(this->homingVelocity, 0.0);
        Serial.printf("Diff in length:\t%f\n", fabs(this->robotData.lengths(i) - goalLengths(i)));
        while (fabs(this->robotData.lengths(i) - goalLengths(i)) > 0.05) {
            this->update();
        }
        this->odrives[i]->setVelocity(0.0, 0.0);
        this->deactivateMotors();
    }
    Serial.println("Pretension setup completed");
}

void CDPR::addPretension() {

    Get_Torques_msg_t torque;
    bool done = false;
    bool check;
    bool torquesSet[NUM_ODRIVES] = {false, false, false, false};
    this->activateMotors();

    Serial.println("Adding pretension");
    while (!done) {
        for (int i = 0; i < NUM_ODRIVES; i++) {
            if (!torquesSet[i]) {
                this->odrives[i]->setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL,
                                                    ODriveInputMode::INPUT_MODE_PASSTHROUGH);
                this->odrives[i]->setTorque(this->tension2Torque(this->tensionSetpoint));
                this->update();
            }
        }
        done = true;
        for (int i = 0; i < NUM_ODRIVES; i++) {
            if (!torquesSet[i]) {
                while (!this->dataStructs[i]->received_torque) {
                    this->update();
                }
                torque = this->dataStructs[i]->last_torque;
                this->dataStructs[i]->received_torque = false;
                check = torque.Torque_Target == this->tension2Torque(this->tensionSetpoint);
                torquesSet[i] = check;
                done = done && check;
            }
        }
        this->checkTorques();
    }
    Serial.println("Added pretension");
}

void CDPR::deactivateMotors() {
    for (int i = 0; i < NUM_ODRIVES; i++) {
        this->confirmSetState(ODriveAxisState::AXIS_STATE_IDLE, i);
    }
    this->completedPretension = false;
    this->hold = false;
}

void CDPR::activateMotors() {
    for (int i = 0; i < NUM_ODRIVES; i++) {
        this->confirmSetState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL, i);
        this->odrives[i]->setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL,
                                            ODriveInputMode::INPUT_MODE_TRAP_TRAJ);
    }
}

void CDPR::update() {
    float t = micros() / 1e6f;
    float dt = t - this->prevUpdateTime;
    this->prevUpdateTime = t;
    static uint32_t lastErrorCheck = 0;
    uint32_t now = millis();
    Get_Encoder_Estimates_msg_t feedback;
    Get_Torques_msg_t torque;
    pumpEventsWrapper(can_intf);

    if (this->robotState == CDPRState::Homed) {
        for (int i = 0; i < NUM_ODRIVES; i++) {
            if (this->dataStructs[i]->received_feedback) {
                feedback = this->dataStructs[i]->last_feedback;
                this->dataStructs[i]->received_feedback = false;
                this->robotData.lengths(i) = this->motorPos2CableLength(feedback.Pos_Estimate, i);
            }
        }
    } else if (this->robotState != CDPRState::Startup) {
        for (int i = 0; i < NUM_ODRIVES; i++) {
            if (this->dataStructs[i]->received_feedback) {
                feedback = this->dataStructs[i]->last_feedback;
                this->dataStructs[i]->received_feedback = false;
                this->robotData.lengths(i) = this->motorPos2CableLength(feedback.Pos_Estimate, i);
            }
            if (this->dataStructs[i]->received_torque) {
                torque = this->dataStructs[i]->last_torque;
                this->dataStructs[i]->received_torque = false;
                this->robotData.tensions(i) = this->torque2Tension(torque.Torque_Estimate);
            }
        }
        this->eePos = this->solveFK(this->prevPos);
        Eigen::Vector2f eeVelRaw = (this->eePos - this->prevPos) / dt;
        float alpha = dt / (this->tau + dt);
        this->eeVel = alpha * eeVelRaw + (1.0f - alpha) * this->eeVel;
        this->prevPos = this->eePos;
    }
    if (this->robotState == CDPRState::Hold || this->robotState == CDPRState::Waypoint) {
        this->applyController(dt);
        if (this->robotState == CDPRState::Waypoint) {
            this->manageWaypoints();
        }
    }


    if (now - lastErrorCheck > 1000) {
        // --- Check ODrive errors for each motor ---
        for (int i = 0; i < NUM_ODRIVES; i++) {
            Get_Error_msg_t errMsg;
            bool success = this->odrives[i]->getError(errMsg, 1);
            if (success) {
                if (errMsg.Active_Errors || errMsg.Disarm_Reason) {
                    Serial.printf("ODrive %d Errors:\n", i);
                    Serial.printf("  Active Errors: 0x%08X\n", errMsg.Active_Errors);
                    Serial.printf("  Disarm Reason: 0x%08X\n", errMsg.Disarm_Reason);
                }
            }
        }
    }

    // if (this->completedHoming) {
    //     for (int i = 0; i < NUM_ODRIVES; i++) {
    //         if (this->dataStructs[i]->received_feedback) {
    //             feedback = this->dataStructs[i]->last_feedback;
    //             this->dataStructs[i]->received_feedback = false;
    //             this->robotData.lengths(i) = this->motorPos2CableLength(feedback.Pos_Estimate, i);
    //         }
    //         // if (this->dataStructs[i]->received_torque) {
    //         //     Get_Torques_msg_t torque = this->dataStructs[i]->last_torque;
    //         //     this->dataStructs[i]->received_torque = false;
    //         //     this->robotData.tensions(i) = this->torque2Tension(torque.Torque_Estimate);
    //         // }
    //     }
    //     if (this->completedPretension) {
    //         this->eePos = this->solveFK(this->eePos);
    //     }
    //     if (this->trajActive) {
    //         this->updateTraj();
    //     }
    //     if (this->hold) {
    //         Eigen::Vector4f lens = this->solveIK(this->holdPos);
    //         for (int i = 0; i < NUM_ODRIVES; i++) {
    //             float motorPos = this->cableLength2MotorPos(lens(i), i);
    //             float motorTorque = this->tension2Torque(this->tensionSetpoint);
    //             this->odrives[i]->setPosition(motorPos, 0.0f, motorTorque);
    //         }
    //     }
    // }
}

void CDPR::setState(CDPRState state) {
    this->robotState = state;
}

CDPRState CDPR::getState() {
    return this->robotState;
}

void CDPR::setGains(float Kp, float Kd, float Ki) {
    this->KpHold = Kp;
    this->KdHold = Kd;
    this->KiHold = Ki;
}

Eigen::Vector2f CDPR::solveFK(Eigen::Vector2f guess, float tol, uint8_t maxIter) {
    
    Eigen::Vector2f eePos = guess;

    for (uint8_t iter = 0; iter < maxIter; iter++) {
        Eigen::Vector4f predLengths;
        Eigen::Matrix<float, 4, 2> J;

        for (int i = 0; i < NUM_ODRIVES; i++) {
            Eigen::Vector2f cornerPos = eePos + this->eeOffsets.row(i).transpose();
            Eigen::Vector2f vec = cornerPos - this->anchorPoints.row(i).transpose();
            float len = vec.norm();
            predLengths(i) = len;

            if (len > 1e-6f) {
                J(i, 0) = vec(0) / len; // dL/dx
                J(i, 1) = vec(1) / len; // dL/dy
            } else {
                J.row(i).setZero();
            }
        }

        // residuals
        Eigen::Vector4f residuals = predLengths - this->robotData.lengths;

        // Newton-Raphson update
        Eigen::Vector2f delta = (J.transpose() * J).ldlt().solve(J.transpose() * residuals);

        eePos -= delta;

        if (delta.norm() < tol) break; // convergence
    }

    return eePos;
}

Eigen::Vector4f CDPR::solveIK(Eigen::Vector2f eePos) {

    Eigen::Vector4f lengths;
    for (int i = 0; i < NUM_ODRIVES; i++) {
        Eigen::Vector2f corner = eePos + this->eeOffsets.row(i).transpose(); // 2x1
        Eigen::Vector2f v = corner - this->anchorPoints.row(i).transpose(); // 2x1
        lengths(i) = v.norm();
    }
    return lengths;
}

float CDPR::motorPos2CableLength(float motorPos, uint8_t motorID) {
    float turnsDiff = this->robotData.motorOffsets(motorID) - motorPos;
    return turnsDiff * this->drumCircumference;
}

float CDPR::cableLength2MotorPos(float cableLength, uint8_t motorID) {
    float turnsDiff = cableLength / this->drumCircumference;
    return this->robotData.motorOffsets(motorID) - turnsDiff;
}

float CDPR::torque2Tension(float torque) {
    return torque / this->drumRadius;
}

float CDPR::tension2Torque(float tension) {
    return tension * this->drumRadius;
}

void CDPR::changeTensionSetpoint(float tensionSetpoint) {
    this->tensionSetpoint = tensionSetpoint;
}

// void CDPR::startTraj(Eigen::Vector2f goal, float speed) {
//     if (speed <= 0.0) return;
//     this->startPos = this->solveFK(this->eePos);
//     this->goalPos = goal;
//     this->trajDuration = (this->goalPos - this->startPos).norm() / speed;
//     this->lastUpdateTime = this->trajStartTime = millis() / 1000.0f;
//     this->trajActive = true;
//     this->hold = false;
// }

// void CDPR::updateTraj() {
//     float t = (millis() / 1000.0f) - this->trajStartTime;
//     float dt = t - this->lastUpdateTime;
//     float s = t / this->trajDuration;

//     if (s >= 1.0f) {
//         s = 1.0f;
//         this->trajActive = false;

//         // Send one final command to hold position and zero velocity
//         Eigen::Vector4f finalLens = this->solveIK(this->goalPos);
//         for (int i = 0; i < NUM_ODRIVES; i++) {
//             float motorPos = this->cableLength2MotorPos(finalLens(i), i);
//             float motorTorque = this->tension2Torque(this->tensionSetpoint);
//             this->odrives[i]->setPosition(motorPos, 0.0f, motorTorque);
//         }

//         this->lastUpdateTime = t;
//         this->holdPos = goalPos;
//         this->hold = true;
//         return;
//     }

//     Eigen::Vector2f desiredPos = startPos + s * (goalPos - startPos);
//     Eigen::Vector4f desiredLens = this->solveIK(desiredPos);

//     Eigen::Vector4f cableDiffs = desiredLens - this->robotData.lengths;
//     Eigen::Vector4f cableSpeeds = cableDiffs / dt;

//     float motorPos, motorSpeed, motorTorque;
//     for (int i = 0; i < NUM_ODRIVES; i++) {
//         motorPos = this->cableLength2MotorPos(desiredLens(i), i);
//         motorSpeed = this->cableLength2MotorPos(cableSpeeds(i), i);
//         motorTorque = this->tension2Torque(this->tensionSetpoint);
//         this->odrives[i]->setPosition(motorPos, motorSpeed, motorTorque);
//     }

//     this->lastUpdateTime = t;
// }

void CDPR::setDesiredPos(Eigen::Vector2f pos) {
    this->intError = Eigen::Vector2f::Zero();
    this->desiredPos = pos;
}

// Eigen::Vector2f CDPR::computeForceFromError() {

//     Eigen::Vector2f dedt = (this->error - this->prevError) / dt;
//     this->prevError = this->error;

//     return this->KpHold * this->error + this->KdHold * this->dedt;
// }

void CDPR::applyController(float dt) {
    Serial.println("=== Controller Active ===");

    // Compute error
    Eigen::Vector2f error = this->desiredPos - this->eePos;
    Serial.printf("EE Pos:       x = %.5f, y = %.5f\n", this->eePos(0), this->eePos(1));
    Serial.printf("Desired Pos:  x = %.5f, y = %.5f\n", this->desiredPos(0), this->desiredPos(1));
    Serial.printf("Error:        x = %.5f, y = %.5f\n", error(0), error(1));

    // Compute derivative
    Eigen::Vector2f velError = this->desiredVel - this->eeVel;
    Serial.printf("EE Vel:    x = %.5f, y = %.5f\n", dedt(0), dedt(1));
    Serial.printf("Desired Vel:  x = %.5f, y = %.5f\n", this->desiredVel(0), this->desiredVel(1));
    Serial.printf("Vel Error:        x = %.5f, y = %.5f\n", velError(0), velError(1));

    // Compute integral
    if (error.norm() < this->holdThresh) {
        this->intError += error * dt;

        // --- Scale integral error if it would violate tension limits ---
        Eigen::Vector2f intForce = this->KiHold * this->intError;
        Eigen::Vector4f intTensions = this->computeTensionsFromForce(intForce);
        float scale = 1.0f;

        for (int i = 0; i < NUM_ODRIVES; i++) {
            if (fabs(intTensions(i) > 1.0e-6f)) {
                if (intTensions(i) > this->maxTension) {
                    scale = fmin(scale, this->maxTension / intTensions(i));
                } else if (intTensions(i) < this->minTension) {
                    scale = fmin(scale, this->minTension / intTensions(i));
                }
            }
        }

        // Apply scaling if needed
        this->intError *= scale;
        Serial.printf("Int Error:    x = %.5f, y = %.5f\n", this->intError(0), this->intError(1));
    }

    // Compute net force
    Eigen::Vector2f netForce = this->KpHold * error + this->KdHold * velError + this->KiHold * this->intError;
    Serial.printf("Net Force:    Fx = %.5f, Fy = %.5f\n", netForce(0), netForce(1));

    // Compute cable tensions
    Eigen::Vector4f tensions = this->computeTensionsFromForce(netForce);
    for (int i = 0; i < NUM_ODRIVES; i++) {
        tensions(i) = fmin(fmax(tensions(i), this->minTension), this->maxTension);
        float torque = this->tension2Torque(tensions(i));
        this->odrives[i]->setTorque(torque);
        Serial.printf("Cable %d: Tension = %.5f N, Torque = %.5f Nm\n", i, tensions(i), torque);
    }
}

void CDPR::applyTrajController() {
    Serial.println("Not ready yet");
}

Eigen::Vector4f CDPR::computeTensionsFromForce(Eigen::Vector2f &force) {
    Eigen::Matrix<float, 4, 2> A = this->computeCableUnitVecs();
    Eigen::Vector4f tensionAdjustments = A * (A.transpose() * A).inverse() * force;
    Eigen::Vector4f tensions = tensionAdjustments.array() + this->tensionSetpoint;
    return tensions.cwiseMax(5.0f);
}

Eigen::Matrix<float, 4, 2> CDPR::computeCableUnitVecs() {
    Eigen::Matrix<float, 4, 2> unitVecs;

    for (int i = 0; i < NUM_ODRIVES; i++) {
        Eigen::Vector2f eeAttachPoint = this->eePos + this->eeOffsets.row(i).transpose();
        Eigen::Vector2f cableVec = this->anchorPoints.row(i).transpose() - eeAttachPoint;
        unitVecs.row(i) = cableVec.normalized();
    }

    return unitVecs;
}

void CDPR::loadSquareTraj(float sideLen, Eigen::Vector2f center) {
    this->waypoints.clear();
    this->waypoints.push_back(Eigen::Vector2f(center(0) + sideLen/2, center(1) + sideLen/2));
    this->waypoints.push_back(Eigen::Vector2f(center(0) - sideLen/2, center(1) + sideLen/2));
    this->waypoints.push_back(Eigen::Vector2f(center(0) - sideLen/2, center(1) - sideLen/2));
    this->waypoints.push_back(Eigen::Vector2f(center(0) + sideLen/2, center(1) - sideLen/2));
    this->waypoints.push_back(Eigen::Vector2f(center(0) + sideLen/2, center(1) + sideLen/2));
}

void CDPR::loadDiamondTraj(float sideLen, Eigen::Vector2f center) {
    this->waypoints.clear();
    float center2CornerDist = sideLen * sqrt(2.0f) / 2.0f;
    this->waypoints.push_back(Eigen::Vector2f(center(0), center(1) + center2CornerDist));
    this->waypoints.push_back(Eigen::Vector2f(center(0) - center2CornerDist, center(1)));
    this->waypoints.push_back(Eigen::Vector2f(center(0), center(1) - center2CornerDist));
    this->waypoints.push_back(Eigen::Vector2f(center(0) + center2CornerDist, center(1)));
    this->waypoints.push_back(Eigen::Vector2f(center(0), center(1) + center2CornerDist));
}

void CDPR::activateWaypoints() {
    this->robotState = CDPRState::Waypoint;
    this->currentWaypointInd = 0;
    this->completedWaypoints = false;
    this->setDesiredPos(this->waypoints[this->currentWaypointInd]);
}

void CDPR::manageWaypoints() {
    if (!this->completedWaypoints) {
        float distThresh = 0.03;
        float speedThresh = 0.01;
        float distError = (this->waypoints[currentWaypointInd] - this->eePos).norm();
        if (distError < distThresh && this->eeVel.norm() < speedThresh) {
            if (this->currentWaypointInd == this->waypoints.size() - 1) {
                this->completedWaypoints = true;
                this->robotState = CDPRState::Hold;
            } else {
                this->currentWaypointInd += 1;
                this->setDesiredPos(this->waypoints[this->currentWaypointInd]);
            }
        }
    }
}

void CDPR::registerCallbacks() {
    for (int i = 0; i < NUM_ODRIVES; i++) {
        this->odrives[i]->onFeedback(onFeedback, this->dataStructs[i]);
        this->odrives[i]->onStatus(onHeartbeat, this->dataStructs[i]);
        this->odrives[i]->onTorques(onTorques, this->dataStructs[i]);
    }
}

void CDPR::confirmSetState(ODriveAxisState desiredState, uint8_t index) {
    while (this->dataStructs[index]->last_heartbeat.Axis_State != desiredState) {
        this->odrives[index]->setState(desiredState);
        this->update();
    }
}

void CDPR::checkODriveConnections() {
    Serial.println("Checking connection to ODrives...");
    for (int i = 0; i < NUM_ODRIVES; i++) {
        Serial.printf("Waiting for ODrive %d\n", i);
        while (!this->dataStructs[i]->received_heartbeat) {
            this->update();
            delay(10);
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

void CDPR::clearODriveErrors() {
    for (int i = 0; i < NUM_ODRIVES; i++) {
        this->odrives[i]->clearErrors();
    }
}