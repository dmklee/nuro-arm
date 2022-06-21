NURO Arm: Accessible Robotics Educational Platform
--------------------------------------------------------------------------------
**[Documentation](https://nuro-arm.readthedocs.io/en/latest/?)** | **[Website](https://dmklee.github.io/nuro-arm/)**

--------------------------------------------------------------------------------

This repository is built to help students learn more about programming and robotics. It aims to provide a high-level interface for performing basic robotic manipulation tasks.  The repo is meant to be accessible: the hardware is low cost; a simulator is provided for those who do not purchase the robot; the api is built to work on Windows, MacOS & Linux.

<p align="center">
  <img src="https://github.com/dmklee/nuro-arm/blob/main/images/xarm.png" height="400"/>
</p>

## Software Installation

### Conda
For Windows, we recommend using a conda environment so that pybullet can be installed
beforehand.
```
conda create -n "myenv" python=3.8
conda activate myenv
conda install -c conda-forge pybullet
pip install nuro-arm
```

### Pip
Using pip will only work for Linux and Mac:
```
pip install nuro-arm
```

#### [Windows Only] Add libraries for handling usb-hid commands:
1. Download this [zip file](https://github.com/libusb/hidapi/releases/download/hidapi-0.10.1/hidapi-win.zip)
2. In File Explorer, navigate into downloaded folder [your downloads location]\hidapi-win\x64
3. Copy the three files ("hidapi.dll","hidapi.lib","hidapi.pdb")
4. Paste them in "C:\Users\[username]\Anaconda3\envs\[your-env-name]\"
#### [Linux Only] Install libraries for usb-hid:
```
sudo apt-get install libhidapi-hidraw0 libhidapi-libusb0
sudo service fwupd stop
```

<a name="robot-kit"></a>
## Robot Kit
### Parts List
You can order the parts for the robot kit easily for around $250.  In order to use the camera, you will need access to a 3D printer (we are working on finding a prebuilt camera holder).  You can use the simulated version of the robot first to see whether you want to purchase the robot.  We have no relationship with the manufacturers of the parts.

Description  | Price 
------------ | ----- 
[Hiwonder xArm Robotic Arm](https://www.amazon.com/LewanSoul-Programmable-Feedback-Parameter-Programming/dp/B0793PFGCY/ref=sr_1_3?dchild=1&keywords=lewansoul+xarm&qid=1618417178&sr=8-3) | $200
[ELP Megapixel 720p USB Camera 100 deg lens](https://www.amazon.com/ELP-megapixel-Camera-Module-120degree/dp/B01DRJXDEA/ref=sr_1_1?crid=12SN0I987B5WH&dchild=1&keywords=elp+megapixel+super+mini+720p+usb+camera+module+with+120degree+lens&qid=1618417242&sprefix=elp+camera+megapix%2Caps%2C157&sr=8-1) | $30
[Small (~1") toy cubes](https://www.amazon.com/ETA-hand2mind-1-inch-Color-Cubes/dp/B01J6GC83U/ref=sr_1_13?dchild=1&keywords=wooden+cubes+color&qid=1619112911&sr=8-13) or find similar items around the house | $19 (or free)
3D Printed parts ([stl files](https://github.com/dmklee/nuro-arm/blob/main/nuro_arm/assets/meshes/)) | ~$10
computer with 2 USB-A ports or equivalent adapters | -

### Assembly
The assembly process takes around 3 hours, including calibration.  Check the documentation for assembly and calibration instructions.

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

## Acknowledgements
The xarm controller code is an amalgam of the following repos:
- https://github.com/ccourson/xArmServoController/
- https://github.com/maximkulkin/lewansoul-lx16a/
- https://gist.github.com/maximecb/7fd42439e8a28b9a74a4f7db68281071
- https://github.com/adeguet1/lewansoul-xarm


## License
This code is released under the [MIT License](LICENSE).
