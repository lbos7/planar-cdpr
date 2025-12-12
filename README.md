# planar-cdpr

Author: Logan Boswell

<p align="center">
  <img src="https://github.com/user-attachments/assets/f2279c7e-81b5-4078-9a42-1a16accd0dd8" width="500"/>
</p>

This is the software for my final project of the Northwestern University MSR program where I built a planar cable-driven parallel robot from scratch that is capable of operating while submerged in water - https://lbos7.github.io/projects/01-cable-driven-parallel-robot.

## Components
- `cdpr_teensy` - contains the code that is uploaded to the teensy using Arduino IDE
- `cdpr_utils` - contains some utility code for communicating with the robot through Python, plotting/analyzing data, and developing/tuning the feedforward model used in the robot control system
- `cdpr_vision` - contains a ROS 2 package that was used for testing the robot's perfomance using apriltags
- `cdpr_msgs` - custom service definitions used for testing robot performance

Note - `cdpr_teensy` is the only folder needed to operate the system; the other folders were only used by me for testing the robot and gathering data to generate plots

## Setup Guide

If you don't have Arduino IDE installed, download and install from here: https://www.arduino.cc/en/software/

Once Arduino IDE is installed, install the ODriveArduino library (https://github.com/odriverobotics/ODriveArduino/tree/master) and install a Teensy-compatible Eigen library port (https://github.com/bolderflight/eigen) by following the process outlined in this guide: https://docs.odriverobotics.com/v/latest/guides/arduino-can-guide.html#installing-the-odrivearduino-library. If they were installed correctly, you should see the folders in the `Arduino/libraries/` directory.

After ensuring the necessary dependencies are installed, copy/move the `cdpr_teensy` folder from this repo into the `Arduino/` directory. From here you can open up the files in Arduino IDE and make edits if necessary. If edits are made, you must recompile the code using the verify button in Arduino IDE, plug in the USB to your computer, and then press the white button in the Teensy to upload the code (a red light on the Teensy should flash a few times if the code is uploaded correctly).

Note - Currently the software is already uploaded to the Teensy that the robot uses, so it's only necessary to reupload code if changes are made.

## Usage Guide

Before using the system, you must complete a few steps:
- open Arduino IDE on your computer and open a serial monitor window (button in the top right)
- plug the 24V, GND, and CAN bus cables into their corresponding connectors on the robot
- plug the power cord into a wall outlet and release the E-Stop (this E-Stop cuts power to the motors when pressed, so you can press it any time you need the motors to stop moving/be unpowered)
- plug the USB cord into your computer

When the USB cord is plugged in, the robot will begin its homing process and then tighten the cables with the end-effector near the center of the workspace.

To operate the system, you can type commands (Next section below) in the top area of the serial monitor window. In the future, another method can be used to send these serial commands such as using Python or LabVIEW.

When you are done using the system, take the following steps:
- press the E-Stop
- unplug the USB cord from your computer
- unplug the power cord from the outlet
- unplug the 24V, GND, and CAN cables

## Serial Command List

- `home` — Run the homing sequence for the robot
- `disable` — Deactivate all motors
- `enable` — Activate all motors
- `debug` — Set robot state to Debug mode
- `reset` — Reset robot: startup, homing, homed state, pretension setup, and activate
- `tension` — Apply pretension to cables
- `setupt` — Perform pretension setup
- `checkt` — Check motor torques
- `checkl` — Check motor positions and cable lengths
- `checkp` — Check end-effector position
- `checks` — Check current robot state
- `checkg` — Check control gains
- `checkf` — Check tensions at the current position
- `gridtest` — Start a grid test routine
- `sets <state>` — Set robot state (`active` or `debug`)
- `setg <Kp> <Kd> <Ki>` — Set control gains Kp, Kd, Ki
- `loads <sideLen> <x> <y>` — Load square waypoints with given side length and center position
- `loadd <sideLen> <x> <y>` — Load diamond waypoints with given side length and center position
- `waypoints [speed]` — Activate waypoints trajectory with optional speed (uses default speed of 1 m/s not provided)
- `move <x> <y> [speed]` — Move to position (x, y) with optional speed (default of 1 m/s)
- `log <x> <y>` — Log the current position at (x, y)