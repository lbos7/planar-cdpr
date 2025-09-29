#ifndef CDPR_CAN_H_
#define CDPR_CAN_H_

#include "ODriveCAN.h"

// Structs for getting data from ODrives
struct ODriveStatus; // hack to prevent teensy compile error
struct ODriveUserData {
    Heartbeat_msg_t last_heartbeat;
    bool received_heartbeat = false;
    Get_Encoder_Estimates_msg_t last_feedback;
    bool received_feedback = false;
    Get_Torques_msg_t last_torque;
    bool received_torque = false;
};

// Feedback function prototypes
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data);
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data);
void onTorques(Get_Torques_msg_t& msg, void* user_data);

#endif  // CDPR_CAN_H_