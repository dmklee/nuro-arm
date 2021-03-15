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
- GUI for calibration and simple arm movements
- IK + joint control (w/ collision avoidance)
- Closed Loop (keypoint) trajectories
- Camera Calibration
- Cube pose estimation
- Testing across platform
- Testing for failure modes/ hardware resets


For getting lobot to show up in /dev/hidraw*
- https://skyboo.net/2018/10/binding-unbinding-usb-drivers-a-k-a-who-stole-my-hidraw1-device-file/
- use udevadm monitor to get the id to bind
- you can check lsusb -t to check that it binds successfull
