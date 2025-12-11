#ifndef CDPR_CAN_H_
#define CDPR_CAN_H_

#include "ODriveCAN.h"

// Structs for getting data from ODrives
struct ODriveStatus; // hack to prevent teensy compile error

/**
 * @struct ODriveUserData
 * @brief Represents feedback data for an ODrive.
 */
struct ODriveUserData {
    Heartbeat_msg_t last_heartbeat;
    bool received_heartbeat = false;
    Get_Encoder_Estimates_msg_t last_feedback;
    bool received_feedback = false;
    Get_Torques_msg_t last_torque;
    bool received_torque = false;
};

// Feedback function prototypes

/**
 * @brief Called every time a Heartbeat message arrives from the ODrive to log data.
 * 
 * @param msg The heartbeat message struct.
 * @param user_data The user data struct that will be updated.
 */
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data);

/**
 * @brief Called every time an Encoder Feedback message arrives from the ODrive to log data.
 * 
 * @param msg The encoder estimates message struct.
 * @param user_data The user data struct that will be updated.
 */
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data);

/**
 * @brief Called every time a Torque message arrives from the ODrive to log data.
 * 
 * @param msg The torque message struct.
 * @param user_data The user data struct that will be updated.
 */
void onTorques(Get_Torques_msg_t& msg, void* user_data);

#endif  // CDPR_CAN_H_