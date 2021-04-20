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

### [Recommended] Install Visual Studio Code
1. Download VS Code for your device by clicking this [link](https://code.visualstudio.com/download)
2. With VS Code open, click the Extensions icon on the left panel.  Under the popular tab, click Install on the Python extension.

<a name="hardware"></a>
## Hardware (estimated time 2.5 hrs)
### Robot
#### Assembly Instructions
The parts needed to assemble the robot will all be located in the Hiwonder box. To assemble the robot, watch the following instructional videos. If you do not have access to pliers or wrenches, you should be able to hold nuts in place by hand (there is no need to tighten anything aggressively).
1. [Assembly 01](https://www.youtube.com/watch?v=68N5oQAYfEI)
2. [Assembly 02](https://www.youtube.com/watch?v=BhTdgkRTBoE)
3. [Assembly 03](https://www.youtube.com/watch?v=ij0365iMALk): the portion 3:25+ is about cable management and is optional. If any wires are sticking out too much, you might want to fasten them down to avoid them getting caught during motion.

#### Calibration
In order to calibrate the robot, you must first complete the [software installation](#software) section.  For safety reasons, you should remove all obstacles from around the robot (including the camera stand if you already set that up) since the obstacle avoidance abilities do not work until it is calibrated.
1. In terminal, navigate to the neu-ro-arm directory
2. Ensure the robot virtual environment is activated
3. Run setup camera script:
	```
	python neu_ro_arm/scripts/setup_xarm.py
	```

### Camera
#### Assembly Instructions
1. Collect the parts shown in the photo below, the 3D printed parts and camera will be in camera box; the rod and screws will be in Hiwonder box.

<p align="center">
  <img src="https://github.com/dmklee/neu-ro-arm/blob/main/neu_ro_arm/data/installation_guide/camera_parts.jpg"/>
</p>

2. Attach the base part to the base plate of the robot. The part should be installed as shown in the picture below.

<p align="center">
  <img src="https://github.com/dmklee/neu-ro-arm/blob/main/neu_ro_arm/data/installation_guide/camera_base.jpg"/>
</p>

3. Install the aluminum rod by sliding it into the base part and then tightening the collar until the rod does not easily move around.

<p align="center">
  <img src="https://github.com/dmklee/neu-ro-arm/blob/main/neu_ro_arm/data/installation_guide/camera_base_rod.jpg"/>
</p>

4. Attach the camera to the camera holder part using the small nuts+bolts.  It is safe to touch the back panel of the camera, but you should avoid scratching it accidentally with your screwdriver.

<p align="center">
  <img src="https://github.com/dmklee/neu-ro-arm/blob/main/neu_ro_arm/data/installation_guide/camera_holder_front.jpg" width="190"/>
  <img src="https://github.com/dmklee/neu-ro-arm/blob/main/neu_ro_arm/data/installation_guide/camera_holder_back.jpg" width="200"/>
</p>

5. Attach the camera holder to the last 3D printed part by tightening the collar around it. Only tighten until snug, as you will adjust the configuration later during calibration.

<p align="center">
  <img src="https://github.com/dmklee/neu-ro-arm/blob/main/neu_ro_arm/data/installation_guide/camera_top.jpg"/>
</p>

6. Place the parts from step 5 onto the aluminum rod, and tighten the collar such that the full assembly is snug in place.

<p align="center">
  <img src="https://github.com/dmklee/neu-ro-arm/blob/main/neu_ro_arm/data/installation_guide/camera_full.jpg"/>
</p>

7.  All done! You are now ready to calibrate the camera.

### Calibration
In order to calibrate the camera, you must first complete the [software installation](#software) section. 
1. In terminal, navigate to the neu-ro-arm directory
2. Ensure the robot virtual environment is activated
3. Run setup camera script:
	```
	python neu_ro_arm/scripts/setup_camera.py
	```

## Cubes
We provide you with some multi-colored cubes.  In order for the camera to identify these cubes, we need to attach Aruco tags. We give you two sheets of Aruco tags (they should look like simple QR codes).  The patterns are designed to be easily identified in the image. 
1. Cut out an individual tag (cut in the white region)
2. Attach the tag to one side of the cube with tape or glue.  If you are using tape, you should avoid taping over the patterned area of the tag; the tape can produce glare which makes it difficult for the camera to see.
3. Repeat for as many cubes as you want to use. Each cube should use a different tag number (this way the camera can differentiate between them).  We provide four copies of 12 different numbered tags so there will be plenty of leftovers.

<p align="center">
  <img src="https://github.com/dmklee/neu-ro-arm/blob/main/neu_ro_arm/data/installation_guide/cubes.jpg"/>
</p>

