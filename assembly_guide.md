# Robotic Kit Assembly Guide
## Table of Contents
1. [Parts List](#parts-list)
2. [Robot Arm](#robot-arm)
3. [Camera](#camera)
4. [Objects](#objects)
5. [FAQ](#faq)

<a name="parts-list"></a>
### Parts List


Description  | Price 
------------ | ----- 
[Hiwonder xArm Robotic Arm](https://www.amazon.com/LewanSoul-Programmable-Feedback-Parameter-Programming/dp/B0793PFGCY/ref=sr_1_3?dchild=1&keywords=lewansoul+xarm&qid=1618417178&sr=8-3) | $200
[ELP Megapixel 720p USB Camera 100 deg lens](https://www.amazon.com/ELP-megapixel-Camera-Module-120degree/dp/B01DRJXDEA/ref=sr_1_1?crid=12SN0I987B5WH&dchild=1&keywords=elp+megapixel+super+mini+720p+usb+camera+module+with+120degree+lens&qid=1618417242&sprefix=elp+camera+megapix%2Caps%2C157&sr=8-1) | $30
[Small (~1") toy cubes](https://www.amazon.com/ETA-hand2mind-1-inch-Color-Cubes/dp/B01J6GC83U/ref=sr_1_13?dchild=1&keywords=wooden+cubes+color&qid=1619112911&sr=8-13) or find similar items around the house | $19 (or free)
3D Printed parts ([stl files](https://github.com/dmklee/nuro-arm/blob/main/nuro_arm/assets/meshes/)) | ~$10
computer with 2 USB-A ports or equivalent adapters | -


<a name="robot-arm"></a>
### Robot Arm
#### Assembly Instructions
The parts needed to assemble the robot will all be located in the Hiwonder box. To assemble the robot, watch the following instructional videos provided by Hiwonder.
1. [Assembly 01](https://www.youtube.com/watch?v=68N5oQAYfEI)
2. [Assembly 02](https://www.youtube.com/watch?v=BhTdgkRTBoE): self-tapping means the screws will have a pointy end.
3. [Assembly 03](https://www.youtube.com/watch?v=ij0365iMALk): it may take some force to fit the blue parts over the servo horns;  the portion after 3:25 is about cable management and is optional. If any wires are sticking out too much, you might want to fasten them down to avoid them getting caught during motion.

The final step is to plug in the robot.  We must plug two things into the controller board on the robot: the power supply and the usb cable.  See the pictures below to understand how to plug things in.  If everything is correct, when you flip the power switch to ON, lights on the motors will turn on.  If you hear a beeping noise, this means the power supply is not plugged in.  Do not leave the robot on for extended periods of time, so make sure to flip the power switch when done.

<p align="center">
  <img src="https://github.com/dmklee/nuro-arm/blob/main/images/installation_guide/arm_plugin_parts.png" width="350"/>
</p>

<p align="center">
  <img src="https://github.com/dmklee/nuro-arm/blob/main/images/installation_guide/arm_plugin_with_text.png" width="500"/>
</p>

#### Calibration
In order to calibrate the robot, you must first <a href="https://github.com/dmklee/nuro-arm/blob/main/README.md#installation">install the software</a>.  For safety reasons, you should remove all obstacles from around the robot (including the camera stand if you already set that up) since the obstacle avoidance abilities do not work until it is calibrated.  Calibration is performed by entering the following command into the terminal.
```
python -m nuro_arm.robot.calibrate
```
You will be guided through the process with several popup windows.

#### General Guidelines
Safety is a priority when working with robots.  While the software is built to prevent collisions or unsafe velocities, you should still be careful to ensure you and the robot are safe.  
- **Safe Workspace**. Before issuing any movement commands, make sure there are no objects within reach of the arm. The robot is only aware of itself, the ground, and the camera so it will not know to avoid other objects.  When you are operating the robot, keep any siblings or pets away.
- **Kill Switch**.  You should be aware that the robot's motion can be stopped at any moment by either unplugging the power cable, or flipping the switch on the control board. 
- **Caring for the robot**. To preserve the lifetime of the robot, unplug it when not in use.  Do not force the robot joints to move as this can ruin the gears in the motors; when the robot is in passive mode, there should not be any resistance to motion.

<a name="camera"></a>
### Camera
#### Assembly Instructions
0. Print the three, 3D printed parts. The stl files can be found here: <a href="https://github.com/dmklee/nuro-arm/blob/main/nuro_arm/assets/meshes/camera_holder.stl">camera-holder</a>, <a href="https://github.com/dmklee/nuro-arm/blob/main/nuro_arm/assets/meshes/top_rod_holder.stl">top-rod-holder</a>, <a href="https://github.com/dmklee/nuro-arm/blob/main/nuro_arm/assets/meshes/base_rod_holder.stl">base-rod-holder</a>. Order the 12" long, 3/8" diameter Aluminum rod ($2.16 at <a href="https://www.mcmaster.com/catalog/127/3983/">McMaster-Carr</a>).

1. Collect the parts shown in the photo below; the rod and screws come with the robot arm as extras.

<p align="center">
  <img src="https://github.com/dmklee/nuro-arm/blob/main/images/installation_guide/camera_parts.jpg"/>
</p>

2. Attach the base-rod-holder part to the base plate of the robot. The part should be installed as shown in the picture below.

<p align="center">
  <img src="https://github.com/dmklee/nuro-arm/blob/main/images/installation_guide/camera_base.jpg"/>
</p>

3. Install the aluminum rod by sliding it into the base part and then tightening the collar until the rod does not easily move around.

<p align="center">
  <img src="https://github.com/dmklee/nuro-arm/blob/main/images/installation_guide/camera_base_rod.jpg"/>
</p>

4. Attach the camera to the camera holder part using the small nuts+bolts.  It is safe to touch the back panel of the camera, but you should avoid scratching it accidentally with your screwdriver.

<p align="center">
  <img src="https://github.com/dmklee/nuro-arm/blob/main/images/installation_guide/camera_holder_front.jpg" width="190"/>
  <img src="https://github.com/dmklee/nuro-arm/blob/main/images/installation_guide/camera_holder_back.jpg" width="200"/>
</p>

5. Attach the camera holder to the last 3D printed part by tightening the collar around it. Only tighten until snug, as you will adjust the configuration later during calibration.

<p align="center">
  <img src="https://github.com/dmklee/nuro-arm/blob/main/images/installation_guide/camera_top.jpg"/>
</p>

6. Place the parts from step 5 onto the aluminum rod, and tighten the collar such that the full assembly is snug in place.

<p align="center">
  <img src="https://github.com/dmklee/nuro-arm/blob/main/images/installation_guide/camera_full.jpg"/>
</p>

7.  All done! Plug in the camera's USB cable into your computer when you are ready to use.

#### Calibration
To calibrate the camera, you will need to print out the <a href="https://github.com/dmklee/nuro-arm/blob/main/images/checkerboard.pdf">checkerboard pattern</a>.  Then, run the following command to calibrate the camera.
```
python -m nuro_arm.camera.calibrate
```
You will be guided through the process with several popup windows.

#### General Guidelines
When not using the camera, use the lens cover to keep it safe.  Ensure that the robot is made aware of the camera in the software for collision detection purposes. The camera circuit board may get hot if used for extended periods of time. 

## Cubes
#### Attaching Aruco Tags
In order for an object to be identified by the camera, it must have a visual tag attached to it. You can generate tags to print out using a provided Python script. An example command which would generate twelve, 25mm wide tags, saving them in current directory, is shown here:
```
python -m nuro_arm.camera.generate_aruco_tags --size 25 --number 12 --destination .
```
Then, simply print out the resulting pdf, cut out the tags you want, and attach a single tag to each cube (see image below as example). It is best if the tags have some white border around them so leave some spacing while cutting. Avoid placing tape directly over the pattern, as this can cause problems due to glare.

<p align="center">
  <img src="https://github.com/dmklee/nuro-arm/blob/main/images/installation_guide/cubes.jpg"/>
</p>

#### General Guidelines
We recommend only using cubes that are less than 5cm wide (so it fits within gripper), and less than 250g (so it does not over-stress the servos).


<a name="faq"></a>
## Frequently Asked Questions
### Do I need to use a camera with the robot arm?
No, the robot arm will work fine without the camera. However, without a camera, you will not be able to locate objects in the scene.

### Can I put Aruco tags on objects that are not cubes?
Yes, but the pose estimate must be corrected to reflect the depth of the novel object.

### Can I use the robot without the camera?
Yes, the robot arm will work fine without the camera, and there is plenty of fun projects to be done with only the robot. However, without a camera, you will not be able to locate objects in the scene.
