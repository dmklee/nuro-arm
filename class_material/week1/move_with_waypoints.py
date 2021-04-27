##############################################################################
# Author: 
# Date: 
#
# The purpose of this script is to record a list of joint positions in by
# passively moving the robot.  Then the list of waypoints will be played by
# by issuing motion commands
#
# The advanced user may figure out how to include arm positions and gripper
# state in the waypoints. Then it is possible to hardcode picking up operations
#
##############################################################################

import time
from neu_ro_arm.robot.robot_arm import RobotArm

def collect_waypoints():
    waypoints = []
    # use inp function to determine when to record the arm joint position
    while True:
        user_input = inp("Store current arm joint position as waypoint? [y/N] : ")
        if user_input == 'y':
            # record joint position and add it to the list of waypoints

        elif user_input == 'N':
            # end the while loop

        else:
            # unknown input
            continue

    return waypoints

def follow_waypoints(waypoints):
    # execute commands sequentially
    # you may want some time delay between commands
    pass


# connecting to the robot
robot_mode = 'real'
robot = RobotArm(robot_mode)

# enter passive mode
robot.passive_mode()

waypoints = collect_waypoints()

# enter active mode to prepare for movement commands
robot.active_mode()

follow_waypoints()
