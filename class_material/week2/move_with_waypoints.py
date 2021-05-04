##############################################################################
# Author: David Klee
# Date: May 4, 2021
#
# The purpose of this script is to record a list of arm joint positions by
# passively moving the robot.  Then the list of waypoints is played back
# by issuing motion commands to the arm.
#
##############################################################################

import time
from neu_ro_arm.robot.robot_arm import RobotArm

def collect_waypoints(robot):
    '''Returns list of arm joint positions that user records
    '''
    # initialize an empty list to store arm joint positions
    waypoints = []

    # use inp function to determine when to record the arm joint position
    while True:
        user_input = input("Store current arm joint position as waypoint? [y/N] : ")
        if user_input == 'y':
            # record joint position and add it to the list of waypoints
            arm_jpos = robot.get_arm_jpos()
            waypoints.append(arm_jpos)
            print('Waypoint recorded.')

        elif user_input == 'N':
            # end the while loop
            print('----- Waypoint collection has ended -----')
            break

        else:
            # unknown input so ignore
            continue

    return waypoints

def follow_waypoints(robot, waypoints):
    '''Issues movement commands to follow list of arm joint positions
    '''
    # allow user to decide when to begin
    # program will sit here until input is received
    input('Press enter when ready to perform movement... ')

    # execute commands sequentially
    for arm_jpos in waypoints:
        # issue movement command to robot
        robot.move_arm_jpos(arm_jpos)

        # you may want a delay between movements
        delay = 1 # seconds
        time.sleep(delay)

# initialize instance of RobotArm
# connects to robot automatically
robot = RobotArm()

# enter passive mode, so arm can be moved by hand
robot.passive_mode()

waypoints = collect_waypoints(robot)

# enter active mode to prepare for movement commands
# at this point, arm should not be moved by hand
robot.active_mode()

follow_waypoints(robot, waypoints)

