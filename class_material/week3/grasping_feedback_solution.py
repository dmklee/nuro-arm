##############################################################################
# Author: David Klee
# Date: May 18, 2021
#
# The purpose of this script is to pick up and move a cube.  The arm motions
# are open loop, and are to be followed in order.  The grasping sequence should
# incorporate feedback such that the program only progresses once the cube is
# held within the gripper.
#
##############################################################################

import time
from neu_ro_arm.robot.robot_arm import RobotArm

# define arm joint position sequences
go_to_block = [[-0.56, -0.03, 0.00, 0.00, 0.0],
               [-0.56, -0.85, 1.64, 0.72, 0.0],
               [-0.56, -1.21, 1.84, 1.05, 0.0],
               [-0.54, -0.01, 1.60, 1.07, 0.0],
               [-0.54,  0.14, 1.59, 1.07, 0.0]]

go_to_dropoff = [[-0.54, -1.20, 1.78, 1.07, 0.0],
                 [ 0.65, -1.20, 1.78, 1.07, 0.0],
                 [ 0.65, -0.15, 1.63, 1.07, 0.0]]

# connect to arm 
robot = RobotArm()

# start at home with gripper open
robot.home()
robot.open_gripper()

# follow sequence to go to cube
for arm_jpos in go_to_block:
    robot.move_arm_jpos(arm_jpos)
    time.sleep(0.5)

################################
# Feedback code starts here
################################

delay = 2 # second
almost_closed = 0.15
while True:
    # attempt to grasp
    robot.close_gripper()

    # recieve feedback about gripper motor position
    # recall 0 is fully closed, 1 is fully open
    gripper_state = robot.get_gripper_state()

    if gripper_state < almost_closed:
        # nothing in gripper
        robot.open_gripper()
        time.sleep(delay)
    else:
        # gripper must have something in it
        print('Cube detected in gripper.')
        break

# What if we wanted to improve this so objects larger than the cube would not
# be moved?

################################
# End of feedback code
################################

# robot should now have a cube
# follow sequence to drop off block
for arm_jpos in go_to_dropoff:
    robot.move_arm_jpos(arm_jpos)
    time.sleep(0.5)

# drop off block
robot.open_gripper()

# return home
robot.home()
