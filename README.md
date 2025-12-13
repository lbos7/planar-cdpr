# planar-cdpr

**Author:** Logan Boswell

<p align="center">
  <img src="https://github.com/user-attachments/assets/f2279c7e-81b5-4078-9a42-1a16accd0dd8" width="500"/>
</p>

This is the software for my final project for the Northwestern University MSR program, where I built a Planar Cable-Driven Parallel Robot (CDPR) from scratch capable of operating while submerged in water.  
More details can be found here: [Project Page](https://lbos7.github.io/projects/01-cable-driven-parallel-robot)

---

## Table of Contents
1. [Components](#components)  
2. [Setup Guide](#setup-guide)  
3. [Usage Guide](#usage-guide)  
4. [Serial Commands](#serial-command-list-case-insensitive)
5. [Notes](#notes)

---

## Components
- `cdpr_teensy` — Contains the code uploaded to the Teensy using Arduino IDE  
- `cdpr_utils` — Utility code for communicating with the robot through Python, plotting/analyzing data, and developing/tuning the feedforward model used in the robot control system; also includes plots and data used for analysis  
- `cdpr_vision` — ROS 2 package used for testing robot performance using AprilTags  
- `cdpr_msgs` — Custom service definitions used for testing robot performance  

**Note:** `cdpr_teensy` is the only folder required to operate the system. The other folders were used for testing and data analysis.

---

## Setup Guide

### 1. Install Software
- Download and install [Arduino IDE](https://www.arduino.cc/en/software/)  
- Install the [ODriveArduino library](https://github.com/odriverobotics/ODriveArduino/tree/master)  (you can follow the proces in the [ODrive Arduino CAN guide](https://docs.odriverobotics.com/v/latest/guides/arduino-can-guide.html#installing-the-odrivearduino-library))
- Install a Teensy-compatible Eigen library port from [bolderflight/eigen](https://github.com/bolderflight/eigen) (you can follow the same process as the ODriveArduino library)

After installation, you should see the libraries in the `Arduino/libraries/` directory.

### 2. Prepare the Project
- Copy or move the `cdpr_teensy` folder from this repo into your `Arduino/` directory  
- Open the project in Arduino IDE and make edits if necessary  
- To upload changes:  
  1. Press the **Verify** button in Arduino IDE to compile  
  2. Connect the USB to your computer  
  3. Press the white button on the Teensy to upload the code (a red light will flash a few times if the upload is successful)

**Note:** The software is already uploaded to the Teensy used by the robot, so re-uploading is only necessary if changes are made.

---

## Usage Guide

### Connecting the Robot
1. Open Arduino IDE and launch a serial monitor window  
2. Connect the 24V, GND, and CAN bus cables to the corresponding connectors on the robot  
3. Connect the power cord to a wall outlet and release the E-Stop (pressing the E-Stop cuts power to the motors immediately)  
4. Connect the USB cord to your computer  

**Startup:** When the USB cord is connected, the robot will begin its homing process. After homing, it will tighten the cables with the end-effector near the center of the workspace.

### Operating the Robot
- You can type commands in the serial monitor window to control the robot (shown below in next section)  
- In the future, commands can also be sent via Python or LabVIEW

### Shutting Down
1. Press the E-Stop  
2. Unplug the USB cord from your computer  
3. Unplug the power cord from the outlet  
4. Disconnect the 24V, GND, and CAN bus cables  


---

## Serial Command List (Case-Insensitive)

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
- `waypoints [speed]` — Activate waypoints trajectory with optional speed (uses default speed of 1 m/s if not provided)
- `move <x> <y> [speed]` — Move to position (x, y) with optional speed (default of 1 m/s); workspace is about 0.8 m x 0.8 m centered at (0, 0), so the commanded position should be between -0.4 m and 0.4 m for both x and y coordinates
- `log <x> <y>` — Log the current position at (x, y), where (x, y) is the estimated end-effector position from and apriltag

## Notes
More information on specific code can be found in specific READMEs in each folder.