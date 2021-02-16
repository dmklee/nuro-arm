# LowCostRobotics

urdf and stl files for braccio robot taken from git@github.com:grassjelly/ros_braccio_urdf.git

these might also be useful:

 - https://github.com/zakizadeh/ros_braccio_moveit/tree/master/config
 - https://github.com/grassjelly/ros_braccio_urdf
    - need to add fixed base link
    - need to fix limits on second gripper joint
 - https://github.com/stefangs/arduino-library-braccio-robot

Steps to get it working
1. Assemble Robot & write it up to arduino
2. Download Arduino on computer
3. Open arduino and set device to UNO and choose port
4. Convert 'arduino-library-braccio-robot' into zip, add to arduino libraries
5. Send braccio_serial example to arduino
6. Download ros_braccio_urdf repo: change second gripper range
7. Download ikpy
8. Run python codes, sending commands over serial

