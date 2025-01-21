# RL PID Server for Raspberry Pi

## Overview

This repository contains a ROS2 action server designed to facilitate the tuning of PID controllers for Encoder motor control on a Raspberry Pi. It offers a flexible framework for testing and optimizing PID parameters, enabling precise motor control for various robotic applications.

The server node communicates with a client node ([rl_pid_uros](https://github.com/knighthawk121/rl_pid_uros.git)) that is responsible for the RL-based PID tuning logic. The client sends desired setpoints (or target positions) and PID gain values to the server, which then executes the PID control loop and returns feedback on the system's performance to the client.

This setup allows for iterative tuning of the PID gains by the RL agent, which can learn optimal parameters based on the feedback received from the server.


## Hardware requirements

* Raspberry pi 3b or later
* Motor with Encoder attached, [Tested on GM20-180SH](https://robu.in/wp-content/uploads/2023/08/18-GM20-180SH-2.pdf)
* Motor Driver [Tested on TB6612FNG](https://www.google.com/search?q=TB6612FNG&sourceid=chrome&ie=UTF-8)
* Some jumper cables
* Power supply (for raspi and driver separately)

```bash
# GPIO mode should be set to 'Board' mode to use this pin definitions.
# Pin Definitions
# Motor A
ENCA = 16 # from the motor ENCA
ENCB = 18 # From the motor ENCB
PWMA = 12 # to the driver pin PWMA  
IN1A = 11 # to the driver pin AO1
IN2A = 13 # to the driver pin AO2

# Motor B
ENCA2 = 29
ENCB2 = 31
PWMB = 35
IN1B = 36
IN2B = 37

STBY = 15  # Shared standby pin
```

## Dependencies

* [Ubuntu Server](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#1-overview) (22.04 or later)
* [ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) (humble or later)
* [rl_pid_uros_interfaces](https://github.com/knighthawk121/rl_pid_uros_interfaces.git) (custom message package for PID tuning actions)
* [RPi.GPIO](https://pypi.org/project/RPi.GPIO/)

## Installation

1. Clone this repository to your Raspberry Pi:

   ```bash
   mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
   git clone https://github.com/knighthawk121/rl_pid_uros_interfaces.git
   git clone https://github.com/knighthawk121/rl_pid_raspi_server.git
   ```

2. Navigate to the repository directory:

   ```bash
   cd ~/ros2_ws
   ```

3. Build the ROS2 package:

   ```bash
   colcon build 
   ```

4. Source the ROS2 workspace:

   ```bash
   source install/setup.bash
   ```

## Usage

### Running the Server Node

To launch the server node, run the following command:
(make sure to source the package)

```bash
# for launching the server directly
ros2 run rl_pid_server pid_server
```

### Running the Test Scripts

The repository also includes several test scripts to validate the motor control and PID implementation:

* **motor\_test.py**:  Basic motor control test script.
* **dual\_motor\_test.py**:  Test script for dual motor control.
* **pid\_test.py**:  Single motor PID control test script.
* **dual\_pid\_test.py**:  Dual motor PID control test script.

To run any of these scripts, use the following command:

```bash
# for launching the menu to use other features
ros2 run rl_pid_server rl_pid_server
```

### Interacting with the Server

The server node exposes a ROS2 action called `/tune_pid_action`. The action definition (in [rl_pid_uros_interfaces](https://github.com/knighthawk121/rl_pid_uros_interfaces.git)) includes:

```bash
# Action definition for PID tuning
# File: rl_pid_uros/action/TunePid.action

# Goal Definition
# These are the parameters sent to start the tuning process
# Each parameter is a floating-point value representing a PID coefficient
float64 kp    # Proportional gain coefficient
float64 ki    # Integral gain coefficient
float64 kd    # Derivative gain coefficient
int64 target_position #specified target position the motor has to reach 0 - 360

---
# Result Definition
# This is sent once when the tuning process completes
# It provides information about the final state of the system
float64 final_error    # The final error value when tuning completed
---
# Feedback Definition
# This is sent periodically during the tuning process
# It provides real-time information about the system's state
float64 current_error      # Current error between target and actual position
int64 current_position    # Current motor position in encoder counts
int64 target_position     # Target position the system is trying to reach
bool motor_stopped     # Motor has reached target/stalled
bool error_stable     # Error has stabilized
float64 velocity      # Current motor velocity
```

The client node can use this action to send PID gain values and target positions to the server, and receive feedback on the motor's performance.


## Contributing

Contributions to this repository are welcome. Please feel free to submit pull requests for bug fixes, new features, or improvements to the documentation. This repo is under developement, and needs some work and fine tuning. 

## License

This project is licensed under the Apache License 2.0.