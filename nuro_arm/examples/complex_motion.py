import time
import numpy as np
import matplotlib.pyplot as plt
import pybullet as pb
import argparse

from nuro_arm.robot.robot_arm import RobotArm

def generate_waypts():
    '''Generates circular path in cartesian space
    '''
    R = 0.06
    num_waypts = 100
    center = np.array((0.19, 0, 0.008))
    angles = np.linspace(0, 2*np.pi, num=num_waypts)
    waypts = np.zeros((N, 3)) + center
    waypts[:,0] += R * np.cos(angles)
    waypts[:,1] += R * np.sin(angles)
    return waypts

def follow_waypts_sim(waypts):
    '''Follow waypts with simulated robot
    '''
    robot = RobotArm('sim',
                     headless=False,
                     realtime=True)

    # move GUI camera to get better view
    yaw, pitch, _, target = pb.getDebugVisualizerCamera()[-4:]
    pb.resetDebugVisualizerCamera(0.75, yaw, pitch, target)

    robot.move_hand_to(waypts[0])
    for i in range(1, len(waypts)):
        pb.addUserDebugLine(waypts[i-1], waypts[i],
                            [0.8,0.3,0.2])
        robot.move_hand_to(waypts[i])

    time.sleep(1)

def follow_waypts_real(waypts):
    '''Follow waypts with real robot

    With real robot, the motion is not smooth if we call
    robot.move_hand_to(...) for each waypt.  This is because
    the movement is monitored for collisions, which introduces
    some delay.  Instead, we need to send move commands directly
    to the controller with robot.controller.move_servos(...) with
    more precise timing between commands.  For this reason, you
    must be careful that the workspace is empty.
    '''
    robot = RobotArm('real')

    # calculate joint positions using ik in advance
    arm_jposs = []
    for waypt in waypts:
        arm_jpos, _ = robot.mp.calculate_ik(waypt)
        arm_jposs.append(arm_jpos)

    robot.move_arm_jpos(arm_jposs[0])
    joint_ids = robot.controller.arm_joint_ids
    for arm_jpos in arm_jposs:
        robot.controller.move_servos(joint_ids, arm_jpos, duration=15)
        time.sleep(0.015)

    time.sleep(1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--sim', action="store_true")
    args = parser.parse_args()

    waypts = generate_waypts()

    if args.sim:
        follow_waypts_sim(waypts)
    else:
        follow_waypts_real(waypts)
