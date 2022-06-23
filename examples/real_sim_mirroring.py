import time
import pybullet as pb

from nuro_arm.robot.robot_arm import RobotArm

# move the arm around by hand and see its state reflected in the simulator gui

robot = RobotArm(headless=False)
robot.passive_mode()

# make GUI view better
pb.resetDebugVisualizerCamera(cameraDistance=1.2,
                              cameraYaw=50,
                              cameraPitch=-40,
                              cameraTargetPosition=(-0.45, 0.35, -0.4))
pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)

while 1:
    robot.mirror_planner()
    time.sleep(0.1)
