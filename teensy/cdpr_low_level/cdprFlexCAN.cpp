#include "cdprFlexCAN.h"
#include "ODriveFlexCAN.hpp"

// FlexCAN object necessary for CAN bus on Teensy 4.1
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

bool setupCan() {
    can_intf.begin();
    can_intf.setBaudRate(CAN_BAUDRATE);
    can_intf.setMaxMB(16);
    can_intf.enableFIFO();
    can_intf.enableFIFOInterrupt();
    can_intf.onReceive(onCanMessage);
    return true;
}

void onCanMessage(const CanMsg& msg) {
    for (auto odrive: odrives) {
       onReceive(msg, *odrive);
    }
}

void pumpEventsWrapper(FlexCAN_T4_Base& can_intf) {
    pumpEvents(can_intf);
}

ODriveCAN createODriveObj(FlexCAN_T4_Base& can_intf, uint8_t node_id) {
    ODriveCAN odrv(wrap_can_intf(can_intf), node_id);
    return odrv;
}