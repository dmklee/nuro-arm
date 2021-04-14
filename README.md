# NEU Robotics Outreach: Robotic Arm

This repository is built to help students learn more about programming and robotics. 
A high-level API is provided for interfacing with a lewansoul xArm robotic arm. 

TODO:
- ~~GUI for calibration and simple arm movements~~
- IK + joint control (w/ collision avoidance)
- Closed Loop (keypoint) trajectories
- Cube pose estimation
- Testing across platform
- Testing for failure modes/ hardware resets
- ~~Get better measurements for urdf file, add gripper~~

## Table of Contents
1. [Installation](#installation)
2. [Features](#features)
3. [Parts List](#parts-list)
4. [Projects](#projects)
4. [Acknowledgements](#acknowledgements)

<a name="installation"></a>
## Installation

<a name="features"></a>
## Features
### Robot [or Simulator]
- Hard offset correction to servos
- Enforce joint limits & maximum velocities
- Forward and Inverse Kinematics
- Collision detection
- GUI for simple joint motion
- Control of gripper

### Camera
- Lens Distortion correction
- Camera ego pose calculation
- Taking photos/videos
- Basic computer vision on imagaes
- Pose estimation of cubes

<a name="parts-list"></a>
## Parts List
This project is built on hardware that is more accessible.  The entire cost of the kit is aorun $250, and items can be ordered conveniently on Amazon.
- [Lewansoul xArm Robotic Arm ($200)](https://www.amazon.com/LewanSoul-Programmable-Feedback-Parameter-Programming/dp/B0793PFGCY/ref=sr_1_3?dchild=1&keywords=lewansoul+xarm&qid=1618417178&sr=8-3)
- [ELP Megapixel 720p USB Camera 100 deg lens ($30)](https://www.amazon.com/ELP-megapixel-Camera-Module-120degree/dp/B01DRJXDEA/ref=sr_1_1?crid=12SN0I987B5WH&dchild=1&keywords=elp+megapixel+super+mini+720p+usb+camera+module+with+120degree+lens&qid=1618417242&sprefix=elp+camera+megapix%2Caps%2C157&sr=8-1)
- Small toy cubes (1" is the default size used here)
- 3D Printed parts (stl files provided in 'src/assets')
- Calibration sheet (printable pdf available in 'src/configs')
- Aruco Tags (printable pdf available in 'src/configs')

<a name="projects"></a>
## Projects
Projects, along with solution scripts, will be released in June 2021.

<a name="acknowledgements"></a>
## Acknowledgements
The xarm controller code is an amalgam of the following repos:
- https://github.com/ccourson/xArmServoController/
- https://github.com/maximkulkin/lewansoul-lx16a/
- https://gist.github.com/maximecb/7fd42439e8a28b9a74a4f7db68281071
- https://github.com/adeguet1/lewansoul-xarm

For getting lobot to show up in /dev/hidraw:
- https://skyboo.net/2018/10/binding-unbinding-usb-drivers-a-k-a-who-stole-my-hidraw1-device-file/ 
- use udevadm monitor to get the id to bind
- you can check lsusb -t to check that it binds successfull

Known Issues:
- (Linux) if hidraw device does not show up after reboot, run "sudo service fwupd stop"
