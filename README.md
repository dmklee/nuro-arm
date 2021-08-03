# NU Robotics Outreach: Robotic Arm

This repository is built to help students learn more about programming and robotics. It aims to provide a high-level interface for performing basic robotic manipulation tasks.  The repo is meant to be accessible: the hardware is low cost; a simulator is provided for those who do not purchase the robot; the api is built to work on Windows, MacOS & Linux.

<p align="center">
  <img src="https://github.com/dmklee/nuro-arm/blob/main/images/xarm.png" height="400"/>
</p>

## Table of Contents
1. [Features](#features)
2. [Installation](#installation)
3. [Robotic Kit](#robot-kit)
5. [Tutorials and Projects](#projects)
6. [Acknowledgements](#acknowledgements)

<a name="installation"></a>
## Software Installation
This package should work on Windows, Mac, or Linux.  See the [Installation Guide](https://github.com/dmklee/nuro-arm/blob/main/installation_guide.md) for details.

<a name="robot-kit"></a>
## Robot Kit
### Assembly
The assembly process takes around 3 hours, including calibration.  Detailed instructions can be found in our <a href="https://github.com/dmklee/nuro-arm/blob/main/assembly_guide.md">Assembly Guide</a>.

<a name="features"></a>
## Features
### Robot [or Simulator]
- Calibration of robot servos
- Enforcement of joint position and velocity limits
- Collision detection
- Forward and Inverse Kinematics
- Support for complex non-linear movements
- GUI for simple joint motion
- GUI for sequencing arm movements

### Camera
- Estimation of intrinsic/extrinsic parameters
- GUI for visualizing or debugging
- Taking photos and videos
- Pose estimation of cubes using Aruco Tags

<a name="projects"></a>
## Projects
Here are some project ideas that are feasible with this platform:
- Face tracking with hand-held camera
- Creating stacks of cubes
- Throwing objects into a bin
- Playing tic-tac-toe
- *Tactile* sensing of objects
- Typing on a keyboard
- Investigating positional accuracy of servos
- Picking up novel objects
If you are an educator looking for more information on projects or want to chat about curriculum ideas, please contact me.

<a name="acknowledgements"></a>
## Acknowledgements
The xarm controller code is an amalgam of the following repos:
- https://github.com/ccourson/xArmServoController/
- https://github.com/maximkulkin/lewansoul-lx16a/
- https://gist.github.com/maximecb/7fd42439e8a28b9a74a4f7db68281071
- https://github.com/adeguet1/lewansoul-xarm
