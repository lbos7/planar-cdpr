# cdpr_teensy

## Overview
This folder contains the Teensy software for the Planar Cable-Driven Parallel Robot (CDPR)  
It handles motor control, serial command processing, and communication with ODrive controllers over CAN  
The software is written in C++ and uses Arduino-style setup and loop in `cdpr_teensy.ino`

---

## File Descriptions

- `cdpr_teensy.ino` — Main entry point for the Teensy software; initializes serial communication, sets up the CDPR object, and runs the main loop
- `cdpr.hpp` — Header file defining the `CDPR` class, robot parameters, control states, and method declarations
- `cdpr.cpp` — Implementation of the core `CDPR` class, including motor control, trajectory generation, state management, and utility functions
- `cdprSerial.h` — Header for serial communication functions; declares `processCommand` and related serial utilities
- `cdprSerial.cpp` — Parses serial commands received over USB or UART and calls the appropriate `CDPR` methods
- `cdprCAN.h` — Header for CAN-based communication interfaces with ODrive motor controllers
- `cdprCAN.cpp` — Implements CAN communication functions for sending/receiving motor commands and telemetry
- `cdprFlexCAN.h` — Header for FlexCAN-specific utilities tailored to Teensy CAN communication
- `cdprFlexCAN.cpp` — Implements the FlexCAN interface for sending/receiving CAN messages on Teensy
- `cdprConfig.h` — Header defining configuration parameters for the robot, such as cable lengths, drum radius, motor IDs, and default gains
- `cdprConfig.cpp` — Implements any runtime configuration loading or initialization for the CDPR parameters

---

## Usage
1. Open the `cdpr_teensy.ino` file in Arduino IDE
2. Connect the Teensy to your PC via USB
3. Build and upload the software
4. Use a serial terminal to send commands to the robot (Commands listed in main [README](../README.md#serial-command-list-case-insensitive))

---

## Notes
- The software relies on **FlexCAN** for CAN communication with ODrive motor controllers
- All robot parameters are defined in `cdprConfig.h` and can be modified for different robot configurations
- The command parser is robust to whitespace and capitalization
