import time

from nuro_arm.robot.robot_arm import RobotArm

robot = RobotArm(headless=False)
robot.passive_mode()

while 1:
    robot._mirror_planner()
    time.sleep(0.1)
