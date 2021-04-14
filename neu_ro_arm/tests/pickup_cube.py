import numpy as np

from src.camera import Camera
from src.camera_utils import find_cubes

from src.robot_arm import RobotArm

if __name__ == "__main__":
    camera = Camera(2)
    robot = RobotArm(camera)
