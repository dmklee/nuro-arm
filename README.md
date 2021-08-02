# NU Robotics Outreach: Robotic Arm

This repository is built to help students learn more about programming and robotics. It aims to provide a high-level interface for performing basic robotic manipulation tasks.  The repo is meant to be accessible: the hardware is low cost; a simulator is provided for those who do not purchase the robot; the api is built to work on Windows, MacOS & Linux.

<p align="center">
  <img src="https://github.com/dmklee/neu-ro-arm/blob/main/images/xarm.png" height="400"/>
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
### Parts List
Description  | Price 
------------ | ----- 
[Hiwonder xArm Robotic Arm](https://www.amazon.com/LewanSoul-Programmable-Feedback-Parameter-Programming/dp/B0793PFGCY/ref=sr_1_3?dchild=1&keywords=lewansoul+xarm&qid=1618417178&sr=8-3) | $200
[ELP Megapixel 720p USB Camera 100 deg lens](https://www.amazon.com/ELP-megapixel-Camera-Module-120degree/dp/B01DRJXDEA/ref=sr_1_1?crid=12SN0I987B5WH&dchild=1&keywords=elp+megapixel+super+mini+720p+usb+camera+module+with+120degree+lens&qid=1618417242&sprefix=elp+camera+megapix%2Caps%2C157&sr=8-1) | $30
[Small (~1") toy cubes](https://www.amazon.com/ETA-hand2mind-1-inch-Color-Cubes/dp/B01J6GC83U/ref=sr_1_13?dchild=1&keywords=wooden+cubes+color&qid=1619112911&sr=8-13) or find similar items around the house | $19 (or free)
3D Printed parts ([stl files](https://github.com/dmklee/neu-ro-arm/blob/main/neu_ro_arm/assets/meshes/)) | ~$10
computer with 2 USB-A ports or equivalent adapters | -

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
