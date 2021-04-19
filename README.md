# NEU Robotics Outreach: Robotic Arm

This repository is built to help students learn more about programming and robotics. It aims to provide a high-level interface for performing basic robotic manipulation tasks.  The repo is meant to be accessible: the hardware is low cost; a simulator is provided for those who do not purchase the robot; the api is built to work on Windows, MacOS & Linux.

TODO:
- ~~GUI for calibration and simple arm movements~~
- ~~IK + joint control (w/ collision avoidance)~~
- ~~Cube pose estimation~~
- Testing across platform
- Testing for failure modes/ hardware resets
- ~~Get better measurements for urdf file, add gripper~~

## Table of Contents
1. [Installation](#installation)
2. [Features](#features)
3. [Parts List](#parts-list)
4. [Tutorials and Projects](#projects)
4. [Acknowledgements](#acknowledgements)

<a name="installation"></a>
## Installation
### Windows
1. Install Anaconda. Download version for Python 8, 64-bit [here](https://www.anaconda.com/products/individual)
2. Open Anaconda Prompt application
3. Clone this repo
    ```
    git clone https://github.com/dmklee/neu-ro-arm.git
    cd neu-ro-arm
    ```
2. Create conda env
    ```
    conda env create -f environment.yml
    ```
3.  Install dependencies for hid api.
    1.  Download hidapi-win.zip from [here](https://github.com/libusb/hidapi/releases)
    2.  Move contents of zip file to "C:\[]\Anaconda3\envs\robot\"

### Linux
1. Install Python 3.6+
3. Clone this repo    
    ```
    git clone https://github.com/dmklee/neu-ro-arm.git
    cd neu-ro-arm
    ```
3. Install dependencies for usb
    ```
    sudo apt-get install libhidapi-hidraw0 libhidapi-libusb0
    ```
5. Install this package
    ```
    pip install .
    ```

<a name="features"></a>
## Features
### Robot [or Simulator]
- Calibration of robot servos
- Enforcement of joint limits & maximum velocities
- Forward and Inverse Kinematics
- Collision detection
- Control of gripper
- GUI for simple joint motion

### Camera
- Estimation of intrinsic/extrinsic parameters
- Camera pose estimation 
- GUI for visualizing or debugging
- Taking photos and videos
- Pose estimation of cubes using Aruco Tags

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
## Tutorials and Projects
This repo was created to facilitate an outreach program for high school students in the Boston area.  All curriculum materials, including project ideas and solutions, will be available upon request starting June 2021.

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
