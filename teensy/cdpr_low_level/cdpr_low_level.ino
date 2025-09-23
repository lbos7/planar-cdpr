#include <Arduino.h>
#include "cdprCAN.h"
#include "cdprFlexCAN.h"

// ODrive node_ids
constexpr uint8_t ODRV0_NODE_ID = 0;
constexpr uint8_t ODRV1_NODE_ID = 1;
constexpr uint8_t ODRV2_NODE_ID = 2;
constexpr uint8_t ODRV3_NODE_ID = 3;

// Instantiate ODrive objects
ODriveCAN odrv0 = createODriveObj(can_intf, ODRV0_NODE_ID);
ODriveCAN odrv1 = createODriveObj(can_intf, ODRV1_NODE_ID);
ODriveCAN odrv2 = createODriveObj(can_intf, ODRV2_NODE_ID);
ODriveCAN odrv3 = createODriveObj(can_intf, ODRV3_NODE_ID);
ODriveCAN* odrives[] = {&odrv0, &odrv1, &odrv2, &odrv3}; // Make sure all ODriveCAN instances are accounted for here

// Keep some application-specific user data for every ODrive.
ODriveUserData odrv0_user_data;
ODriveUserData odrv1_user_data;
ODriveUserData odrv2_user_data;
ODriveUserData odrv3_user_data;

bool on = false;
float prev_pos = 0.0;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);

  // Wait for up to 3 seconds for the serial port to be opened on the PC side.
  // If no PC connects, continue anyway.
  for (int i = 0; i < 30 && !Serial; ++i) {
    delay(100);
  }
  delay(200);


  Serial.println("Starting ODriveCAN demo");

  // Register callbacks for the heartbeat and encoder feedback messages
  odrv0.onFeedback(onFeedback, &odrv0_user_data);
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);

  // Configure and initialize the CAN bus interface. This function depends on
  // your hardware and the CAN stack that you're using.
  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true); // spin indefinitely
  }

  Serial.println("Waiting for ODrive...");
  while (!odrv0_user_data.received_heartbeat) {
    pumpEventsWrapper(can_intf);
    delay(100);
  }

  Serial.println("found ODrive");

  // request bus voltage and current (1sec timeout)
  Serial.println("attempting to read bus voltage and current");
  Get_Bus_Voltage_Current_msg_t vbus;
  if (!odrv0.request(vbus, 1000)) {
    Serial.println("vbus request failed!");
    while (true); // spin indefinitely
  }

  Serial.print("DC voltage [V]: ");
  Serial.println(vbus.Bus_Voltage);
  Serial.print("DC current [A]: ");
  Serial.println(vbus.Bus_Current);

  Serial.println("Enabling closed loop control...");
  while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrv0.clearErrors();
    delay(1);
    odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    // Pump events for 150ms. This delay is needed for two reasons;
    // 1. If there is an error condition, such as missing DC power, the ODrive might
    //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
    //    on the first heartbeat response, so we want to receive at least two
    //    heartbeats (100ms default interval).
    // 2. If the bus is congested, the setState command won't get through
    //    immediately but can be delayed.
    for (int i = 0; i < 15; ++i) {
      delay(10);
      pumpEventsWrapper(can_intf);
    }
  }

  Serial.println("ODrive running!");

  Get_Encoder_Estimates_msg_t feedback = odrv0_user_data.last_feedback;
  odrv0_user_data.received_feedback = false;
  prev_pos = feedback.Pos_Estimate;

  delay(10000);
}

void loop() {

  if (on) {
    pumpEventsWrapper(can_intf); // This is required on some platforms to handle incoming feedback CAN messages
                          // Note that on MCP2515-based platforms, this will delay for a fixed 10ms.
                          //
                          // This has been found to reduce the number of dropped messages, however it can be removed
                          // for applications requiring loop times over 100Hz.

    float SINE_PERIOD = 2.0f; // Period of the position command sine wave in seconds

    float t = 0.001 * millis();
    
    float phase = t * (TWO_PI / SINE_PERIOD);

    odrv0.setPosition(
      sin(phase), // position
      cos(phase) * (TWO_PI / SINE_PERIOD) // velocity feedforward (optional)
    );

    // print position and velocity for Serial Plotter
    if (odrv0_user_data.received_feedback) {
      Get_Encoder_Estimates_msg_t feedback = odrv0_user_data.last_feedback;
      odrv0_user_data.received_feedback = false;
      Serial.print("odrv0-pos:");
      Serial.print(feedback.Pos_Estimate);
      Serial.print(",");
      Serial.print("odrv0-vel:");
      Serial.println(feedback.Vel_Estimate);
    }
  
  } else {
    odrv0.setPosition(
      prev_pos, // position
      0 // velocity feedforward (optional)
    );
  }
    
  if (Serial.available()) {

    String msg = Serial.readString();
    msg.trim();

    if (msg.equals("on")) {
      on = true;
      odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    } else if (msg.equals(("off"))) {
      on = false;
      odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);
    }

  }
}