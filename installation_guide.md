## Table of Contents
1. [Software](#software)
2. [Hardware](#hardware)

<a name="software"></a>
## Software (estimated time 15 min)

### Install Anaconda
1. Download the Python 3.8 64-Bit Graphical Installer from this [link](https://www.anaconda.com/products/individual)
2. Open downloaded file and click thru the Installer, selecting the default options.

### Install Git
1. Open a terminal application (for Mac it is called Terminal, for Windows it is Anaconda Prompt)
2. Enter the following command and hit enter:
	```
	conda install -c anaconda git
	```

### Install API and Create Virtual Environment
1. Open terminal application and enter the following command:
	```
	git clone https://github.com/dmklee/neu-ro-arm.git
	```
2. Move into the repository directory with this command:
	```
	cd neu-ro-arm/
	```
3. Create the virtual environment:
    1. [**Windows Only**]
        ```
        conda env create -f environment_win.yml
        ```
    2. [**Mac Only**]
        ```
        conda env create -f environment_mac.yml
        ```
4. Activate the virtual environment:
	```
	conda activate robot
	```
4. Install Robot API so it exists in Python path:
	```
	pip install .
	```
5. [**Windows only**] Add libraries for handling usb-hid commands:
	1. Download this [zip file])https://github.com/libusb/hidapi/releases/download/hidapi-0.10.1/hidapi-win.zip)
	2. In File Explorer, navigate into downloaded folder {your downloads location}\hidapi-win\x64
	3. Copy the three files ("hidapi.dll","hidapi.lib","hidapi.pdb")
	4. Paste them in "C:\Users\[username]\Anaconda3\envs\robot\"

5. [**Linux only**] Install libraries for usb-hid:
	1. In terminal, enter following command:
    ```
    sudo apt-get install libhidapi-hidraw0 libhidapi-libusb0
    ```
	2. You may need to run the following command so that the robot can be connected
	```
	sudo service fwupd stop
	```

### [Recommended] Install Visual Studio Code
1. Download VS Code for your device by clicking this [link](https://code.visualstudio.com/download)
2. With VS Code open, click the Extensions icon on the left panel.  Under the popular tab, click Install on the Python extension.

<a name="hardware"></a>
## Hardware (estimated time 2.5 hrs)
### Robot
#### Assembly Instructions
The parts needed to assemble the robot will all be located in the Hiwonder box. To assemble the robot, watch the following instructional videos. If you do not have access to pliers or wrenches, you should be able to hold nuts in place by hand (there is no need to tighten anything aggressively).
1. [Assembly 01](https://www.youtube.com/watch?v=68N5oQAYfEI)
2. [Assembly 02](https://www.youtube.com/watch?v=BhTdgkRTBoE): self-tapping means the screws will have a pointy end.
3. [Assembly 03](https://www.youtube.com/watch?v=ij0365iMALk): it may take some force to fit the blue parts over the servo horns;  the portion after 3:25 is about cable management and is optional. If any wires are sticking out too much, you might want to fasten them down to avoid them getting caught during motion.

#### Calibration
In order to calibrate the robot, you must first complete the [software installation](#software) section.  For safety reasons, you should remove all obstacles from around the robot (including the camera stand if you already set that up) since the obstacle avoidance abilities do not work until it is calibrated.
1. In terminal, navigate to the neu-ro-arm directory
2. Ensure the robot virtual environment is activated
	```
	conda activate robot
	```
3. Run setup xarm script:
	```
	python neu_ro_arm/scripts/setup_xarm.py
	```
The setup xarm script will calibrate the robot.  Make sure that no objects are nearby when calibrating, so if you have already installed the camera, remove the rod temporarily.
1. The first step of calibration is to move the robot to its "HOME" state, which is pictured below.  The robot should be in passive mode and you want to make any modifications such that it is pointing straight up and the base is aligned.  You can ignore the gripper position.

<p align="center">
  <img src="https://github.com/dmklee/neu-ro-arm/blob/main/images/installation_guide/arm_home_position_front.jpg" width="250"/>
  <img src="https://github.com/dmklee/neu-ro-arm/blob/main/images/installation_guide/arm_home_position_side.jpg" width="250"/>
</p>

2. The next step is to move the arm into a bent position as shown below. The robot will be in passive mode so there should be no resistance.  The gripper position can be ignored.  This step allows us to determine which way the servos were installed.

<p align="center">
  <img src="https://github.com/dmklee/neu-ro-arm/blob/main/images/installation_guide/arm_motor_calibration.jpg" width="150"/>
</p>

3. Now we will calibrate the grippers.  You will be asked to move the gripper fingers to the closed then open position. The grippers should be easy to move: use two hands and do not try to squeeze them together. For the open position aim for a configuration as shown below. 

<p align="center">
  <img src="https://github.com/dmklee/neu-ro-arm/blob/main/images/installation_guide/gripper_closed.jpg"/>
</p>

#### General Guidelines
Safety is a priority when working with robots.  While the software is built to prevent collisions or unsafe velocities, you should still be careful to ensure you and the robot are safe.  
- **Safe Workspace**. Before issuing any movement commands, make sure there are no objects within reach of the arm. The robot is only aware of itself, the ground, and the camera so it will not know to avoid other objects.  When you are operating the robot, keep any siblings or pets away.
- **Kill Switch**.  You should be aware that the robot's motion can be stopped at any moment by either unplugging the power cable, or flipping the switch on the control board. 
- **Caring for the robot**. To preserve the lifetime of the robot, unplug it when not in use.  Do not force the robot joints to move as this can ruin the gears in the motors; when the robot is in passive mode, there should not be any resistance to motion.


### Camera
#### Assembly Instructions
1. Collect the parts shown in the photo below, the 3D printed parts and camera will be in camera box; the rod and screws will be in Hiwonder box.

<p align="center">
  <img src="https://github.com/dmklee/neu-ro-arm/blob/main/images/installation_guide/camera_parts.jpg"/>
</p>

2. Attach the base part to the base plate of the robot. The part should be installed as shown in the picture below.

<p align="center">
  <img src="https://github.com/dmklee/neu-ro-arm/blob/main/images/installation_guide/camera_base.jpg"/>
</p>

3. Install the aluminum rod by sliding it into the base part and then tightening the collar until the rod does not easily move around.

<p align="center">
  <img src="https://github.com/dmklee/neu-ro-arm/blob/main/images/installation_guide/camera_base_rod.jpg"/>
</p>

4. Attach the camera to the camera holder part using the small nuts+bolts.  It is safe to touch the back panel of the camera, but you should avoid scratching it accidentally with your screwdriver.

<p align="center">
  <img src="https://github.com/dmklee/neu-ro-arm/blob/main/images/installation_guide/camera_holder_front.jpg" width="190"/>
  <img src="https://github.com/dmklee/neu-ro-arm/blob/main/images/installation_guide/camera_holder_back.jpg" width="200"/>
</p>

5. Attach the camera holder to the last 3D printed part by tightening the collar around it. Only tighten until snug, as you will adjust the configuration later during calibration.

<p align="center">
  <img src="https://github.com/dmklee/neu-ro-arm/blob/main/images/installation_guide/camera_top.jpg"/>
</p>

6. Place the parts from step 5 onto the aluminum rod, and tighten the collar such that the full assembly is snug in place.

<p align="center">
  <img src="https://github.com/dmklee/neu-ro-arm/blob/main/images/installation_guide/camera_full.jpg"/>
</p>

7.  All done! You are now ready to calibrate the camera.

#### Calibration
In order to calibrate the camera, you must first complete the [software installation](#software) section. 

To perform the calibration, follow the following steps. 
1. In terminal, navigate to the neu-ro-arm directory
2. Ensure the robot virtual environment is activated
	```
	conda activate robot
	```
3. Run setup camera script:
	```
	python neu_ro_arm/scripts/setup_camera.py
	```

The setup camera script will first perform a scan over available cameras, prompting you to select the one used by the robot.  Next, you will be asked to place the checkerboard pattern in front of the robot and position the camera to view it.  Pictures illustrating the calibration and the proper camera view are shown below.
<p align="center">
  <img src="https://github.com/dmklee/neu-ro-arm/blob/main/images/installation_guide/camera_calibration_setup.jpg" width="250"/>
  <img src="https://github.com/dmklee/neu-ro-arm/blob/main/images/installation_guide/camera_calibration_view.png" width="250"/>
</p>

#### General Guidelines
When not using the camera, use the lens cover to keep it safe.  Ensure that the robot is made aware of the camera in the software for collision detection purposes (we will walk you through this). The camera circuit board may get hot if used for extended periods of time. 

## Cubes
We provide you with some multi-colored cubes.  In order for the camera to identify these cubes, we need to attach Aruco tags. We give you two sheets of Aruco tags (they should look like simple QR codes).  The patterns are designed to be easily identified in the image. 
1. Cut out an individual tag (cut in the white region)
2. Attach the tag to one side of the cube with tape or glue.  If you are using tape, you should avoid taping over the patterned area of the tag; the tape can produce glare which makes it difficult for the camera to see.
3. Repeat for as many cubes as you want to use. **Each cube should use a different tag number** (this way the camera can differentiate between them).  We provide four copies of 12 different numbered tags so there will be plenty of leftovers.

<p align="center">
  <img src="https://github.com/dmklee/neu-ro-arm/blob/main/images/installation_guide/cubes.jpg"/>
</p>

