##############################################################################
# Author: David Klee
# Date: May 11, 2021
#
# The purpose of this script is to record a list of arm joint & gripper
# positions by passively moving the robot.  Then the list of waypoints is
# played back by issuing motion commands to the robot.
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
            gripper_state = robot.get_gripper_state()

            ######################################################
            full_robot_state = arm_jpos
            full_robot_state.append(gripper_state)

            waypoints.append(full_robot_state)
            ######################################################
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
    for robot_state in waypoints:
        # issue movement command to robot

        ######################################################
        arm_jpos = robot_state[:-1]
        gripper_state = robot_state[-1]

        robot.move_arm_jpos(arm_jpos)
        robot.set_gripper_state(gripper_state)
        ######################################################

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

