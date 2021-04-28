##############################################################################
# Author: David Klee
# Date: 4.27.21
#
# The purpose of this script is to print measurements of the robots arm joint
# positions, while it is allowed to be passively moved around.  The printing
# will continue until the robot gripper is closed by the user
#
##############################################################################
import time
from neu_ro_arm.robot.robot_arm import RobotArm
from neu_ro_arm.constants import GRIPPER_CLOSED

# Connect to the robot
robot_mode = 'real'
robot = RobotArm(robot_mode)

# ensure gripper is open to start
robot.open_gripper()

# enter passive mode so motors move freely
robot.passive_mode()

# loop will run until gripper is closed manually
while True:
    # get arm joint positions in radians
    arm_jpos = robot.get_arm_jpos()

    # log the joint positions to the terminal
    for i in range(len(arm_jpos)):
        joint_name = robot.joint_names[i]
        joint_pos = arm_jpos[i]
        print(f"{joint_name} : {joint_pos:.2f} radians")

    print()

    # check for termination condition
    gripper_state = robot.get_gripper_state()
    if gripper_state < GRIPPER_CLOSED + 0.01:
        break

    # add some delay so the while loop doesnt run too fast
    delay = 1 # seconds
    time.sleep(delay)

print('Program is ending')

