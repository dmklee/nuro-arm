import numpy as np
import pybullet as pb
import pybullet_data

from abc import ABC

class MotionPlanner():
    '''Use pybullet to handle IK, and collision checking'''
    URDF_FILE = "robot/xarm.urdf"
    def __init__(self, client=pb.DIRECT):
        self.client = pb.connect(client)

        # this path is where we find platform
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.loadURDF('plane.urdf', [0,0,0])

        # add xarm urdf
        self.id = pb.loadURDF(self.URDF_FILE, [0,0,0],[0,0,0,1])
        self.end_effector_index = 5

    def calculate_ik(self, pos, rot=None):
        if rot is not None and len(rot) == 3:
            rot = pb.getQuaternionFromEuler(rot)
        return pb.calculateInverseKinematics(self.id,
                                             self.end_effector_index,
                                             pos, rot)

    def get_end_effector_pos(self, jpos):
        self._teleport_to_jpos(jpos)
        state = pb.getLinkState(self.id, self.end_effector_index)
        return state[4]

    def get_end_effector_rot(self, jpos):
        self._teleport_to_jpos(jpos)
        state = pb.getLinkState(self.id, self.end_effector_index)
        return pb.getEulerFromQuaternion(state[5])

    def _teleport_to_jpos(self, jpos):
        [pb.resetJointState(self.id, i, jp) for i,jp in enumerate(jpos)]

class XArmBase(ABC):
    def close_gripper(self):
        pass

    def open_gripper(self):
        pass

    def move_gripper(self, pos, rot=None):
        pass

    def move_to_jpos(self, jpos):
        pass

class Robot(XArmBase):
    pass

class Simulator():
    def __init__(self):
        self._pb_sim = MotionPlanner(pb.GUI)

    def move_to_jpos(self, jpos):
        pass

    def close_gripper(self):
        pass

    def 

if __name__ == "__main__":
    robot = MotionPlanner()
    print(robot.calculate_ik((1.,0,0)))
