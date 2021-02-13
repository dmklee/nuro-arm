import numpy as np
import numpy.random as npr

import pybullet as pb
import pybullet_data

class Braccio:
    def __init__(self):
        #define
        self.joint_limits
        self.home_joint_angles
        self.urdf_file_path
        self.joint_names

    def initialize(self):
        '''
        set up pybullet simulator
        load robot urdf
        set joint angles to home
        '''
        pass

    def go_home(self):
        '''
        move to home position
        '''
        pass

    def go_to(self, pos, rot):
        pass

    def move_joints_absolute(self, joint_angles):
        pass

    def move_joints_relative(self, joint_displacement):
        pass

    def _calculate_IK(self, pos, rot):
        '''
        calculate joint angles given end effector position and rotation
        '''
        pass

    def _get_joint_state(self, joint_name):
        pass

    def _send_command(self, command):
        pass



