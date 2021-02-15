import numpy as np
import os

import pybullet as pb
import pybullet_data
from abc import abstractmethod

def toDeg(angle):
    '''Convert radians to degrees'''
    return 180. * angle / np.pi

def toRad(angle):
    '''Convert degrees to radians'''
    return np.pi * angle / 180.

class Robot:
    def __init__(self, urdf_filepath, render=True):
        self.urdf_filepath = urdf_filepath
        self.home_joint_states = toRad(np.array((90,90,90,90,90,73,73)))
        self.max_forces = np.array((50,30,30,30,30,30,30))
        self.joint_names = ['base', 'shoulder', 'elbow', 'wristRotation',
                            'wrist', 'gripper_left', 'gripper_right']
        self.arm_joint_indices = (0,1,2,3,4)
        self.gripper_joint_indices = (5,6)
        self.arm_joint_limits = None
        self.gripper_joint_limits = (0, 0.3)
        if render:
            self.client = pb.connect(pb.GUI)
        else:
            self.client = pb.connect(pb.DIRECT)
        self.initialize()
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        # pb.setGravity(0,0,-10)
        self.plane_id = pb.loadURDF('plane.urdf', [0,0,0])



    def initialize(self):
        '''
        Load robot from urdf
        set joint angles to home
        '''
        self.id = pb.loadURDF(self.urdf_filepath, [0,0,0], [0,0,0,1])
        self.end_effector_id = 5
        [pb.resetJointState(self.id, idx, state) for idx,state in
         enumerate(self.home_joint_states)]

    def open_gripper(self):
        p1, p2 = self._get_gripper_joint_states()
        self._send_gripper_command(self.gripper_joint_limits[1])
        it = 0
        while not np.isclose(p1, self.gripper_joint_limits[1]):
            pb.stepSimulation()
            it += 1
            if it > 100:
                return False
            p1, p2 = self._get_gripper_joint_states()
        return True

    def close_gripper(self):
        ''''''
        p1, p2 = self._get_gripper_joint_states()
        limit = self.gripper_joint_limits[0]
        self._send_gripper_command(limit)
        # self._sendGripperCloseCommand()
        it = 0
        while not (np.isclose(p1, self.gripper_joint_limits[1]) and
                   np.isclose(p2, self.gripper_joint_limits[1])):
            pb.stepSimulation()
            it += 1
            p1_, p2_ = self._get_gripper_joint_states()
            if it > 100 or (abs(p1-p1_)<0.0001 and abs(p2-p2_)<0.0001):
                mean = (p1+p2)/2 + 0.01
                self._send_gripper_command(mean)
                return False
            p1 = p1_
            p2 = p2_
        return True

    def get_joint_states(self):
        pass

    def _get_gripper_joint_states(self):
        return (np.pi/2 - pb.getJointState(self.id, self.gripper_joint_indices[0])[0],
                pb.getJointState(self.id, self.gripper_joint_indices[1])[0] - np.pi/2)

    def _calculate_IK(self, pos, rot):
        '''
        pos: vec3 <x,y,z>
        rot: vec4 quaternion <x,y,z,w>
        '''
        return pb.calculateInverseKinematics(self.id,
                                             self.end_effector_id,
                                             pos,
                                             rot)


    def _send_gripper_command(self, target_pos):
        pb.setJointMotorControlArray(
            self.id, self.gripper_joint_indices, pb.POSITION_CONTROL,
            targetPositions=[np.pi/2-target_pos, np.pi/2-target_pos],
            forces=self.max_forces[-2:],
            positionGains=[0.02]*2, velocityGains=[1.0]*2
        )

    def _send_arm_command(self, target_pos):
        num_servos = len(self.arm_joint_indices)
        pb.setJointMotorControlArray(
            self.id, self.arm_joint_indices, pb.POSITION_CONTROL,
            targetPositions=target_pos,
            forces=self.max_forces[:-2],
            positionGains=[0.02]*num_servos, velocityGains=[1.0]*num_servos
        )

    # @abstractmethod
    # def _send_command(self, command):
        # pass

if __name__ == "__main__":
    import time
    filepath = 'ros_braccio_urdf/urdf/braccio_arm.urdf'
    r = Robot(filepath)
    while True:
        r.close_gripper()
        time.sleep(0.5)
        r.open_gripper()
        time.sleep(0.5)
