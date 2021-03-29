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
        pb.loadURDF('plane.urdf', [0,0.5,0])

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
    def __init__(self, mode):
        assert mode in ('robot', 'simulator')
        pb_client = {'robot' : pb.DIRECT,
                     'simulator' : pb.GUI,
                    }[mode]

        self._pb_sim = MotionPlanner(pb_client)

    def close_gripper(self) -> float:
        pass

    def open_gripper(self) -> float:
        pass

    def move_gripper(self, pos, rot=None) -> bool:
        pass

    def move_to_jpos(self, jpos) -> bool:
        """Move to a joint angle
        """
        pass

    def get_gripper_pos(self):
        pass

    def get_jpos(self):
        pass

class Robot(XArmBase):
    def gui(self):
        """Control each joint with GUI"""
        return

    def calibrate(self):
        """
        Do HOME position by setting offsets in hardware
        Handle any additional issues with software constraints on joint angles
        Save all this in config file
        Get gripper close/open positions
        """
        pass

class Simulator(XArmBase):
    GRIPPER_CLOSED = -0.2
    GRIPPER_OPENED = 0.5
    def __init__(self):
        super().__init__('simulator')

    def close_gripper(self) -> float:
        pass

    def open_gripper(self) -> float:
        pass

    def move_gripper(self, pos, rot=None) -> bool:
        pass

    def move_to_jpos(self, jpos) -> bool:
        """Move to a joint angle
        """
        pass

    def get_gripper_pos(self):
        pass

    def get_jpos(self):
        pass

if __name__ == "__main__":
    import time
    robot = Simulator()
    while True:
        time.sleep(0.1)
