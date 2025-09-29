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
bool setupCan();
void onCanMessage(const CanMsg& msg);
void pumpEventsWrapper(FlexCAN_T4_Base& can_intf);
ODriveCAN createODriveObj(FlexCAN_T4_Base& can_intf, uint8_t node_id);

#endif  // CDPR_FLEXCAN_H_