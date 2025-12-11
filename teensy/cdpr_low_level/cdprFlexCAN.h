#ifndef CDPR_FLEXCAN_H_
#define CDPR_FLEXCAN_H_

#include "ODriveCAN.h"
#include <FlexCAN_T4.h>
#include "cdprConfig.h"

// Declarations to avoid compilation errors
struct CAN_message_t;
using CanMsg = CAN_message_t;

// External variables for FlexCAN_T4 object & ODriveCAN array
extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;
extern ODriveCAN* odrives[NUM_ODRIVES];

// Function prototypes

/**
 * @brief Sets up CAN communication with teensy and ODrives.
 * 
 * @return true CAN communication was set up successfully.
 * @return false CAN communication was not set up successfully.
 */
bool setupCan();

/**
 * @brief Called for every message that is received on the CAN bus.
 * 
 * @param msg The received CAN message.
 */
void onCanMessage(const CanMsg& msg);

/**
 * @brief Wrapper for calling the pumpEvents() function.
 * 
 * @param can_intf The FlexCAN_T4 object necessary for CAN communication.
 */
void pumpEventsWrapper(FlexCAN_T4_Base& can_intf);

/**
 * @brief Creates ODrive objects.
 * 
 * @param can_intf The FlexCAN_T4 object necessary for CAN communication.
 * @param node_id The ID # for the ODrive.
 * @return ODriveCAN The created ODriveCAN object.
 */
ODriveCAN createODriveObj(FlexCAN_T4_Base& can_intf, uint8_t node_id);

#endif  // CDPR_FLEXCAN_H_