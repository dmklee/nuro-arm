import numpy as np
import pybullet as pb
import pybullet_data

class XArmMotionPlanner():
    '''Use pybullet to handle IK, and collision checking'''
    URDF_FILE = "robot/xarm.urdf"
    def __init__(self, ):
        self.client = pb.connect(pb.GUI)

        # this path is where we find platform
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.table_id = pb.loadURDF('plane.urdf', [0,0,0])

        # add xarm urdf
        self.robot_id = pb.loadURDF(self.URDF_FILE, [0,0,0],[0,0,0,1])
        print(pb.getNumJoints(self.robot_id))
        for i in range(pb.getNumJoints(self.robot_id)):
            print(pb.getJointInfo(self.robot_id, i))

if __name__ == "__main__":
    robot = XArmMotionPlanner()
    import time
    while True:
        time.sleep(0.1)

