# learning robotics

This repository is built to help students learn more about programming and robotics. 
A high-level API is provided for interfacing with a lewansoul xArm robotic arm. 

Core Functionality:
- Robotic Arm calibration
- Camera calibration
- Joint & End-effector positional control of the arm
- Access to simulated enviroment
- Pose Estimation for simple blocks

TODO:
- ~~GUI for calibration and simple arm movements~~
- IK + joint control (w/ collision avoidance)
- Closed Loop (keypoint) trajectories
- Camera Calibration
- Cube pose estimation
- Testing across platform
- Testing for failure modes/ hardware resets
- Get better measurements for urdf file, add gripper, fix inertia errors


For getting lobot to show up in /dev/hidraw:
- https://skyboo.net/2018/10/binding-unbinding-usb-drivers-a-k-a-who-stole-my-hidraw1-device-file/ 
- use udevadm monitor to get the id to bind
- you can check lsusb -t to check that it binds successfull

This repo is built using methods developped by others:
https://github.com/ccourson/xArmServoController/
https://github.com/maximkulkin/lewansoul-lx16a/
https://gist.github.com/maximecb/7fd42439e8a28b9a74a4f7db68281071
https://github.com/adeguet1/lewansoul-xarm

Known Issues:
- (Linux) if hidraw device does not show up after reboot, run "sudo service fwupd stop"
