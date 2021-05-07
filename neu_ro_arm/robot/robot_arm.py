import numpy as np

import matplotlib
matplotlib.use('TkAgg')
import tkinter as tk

from neu_ro_arm.constants import GRIPPER_CLOSED, GRIPPER_OPENED
from neu_ro_arm.robot.motion_planner import (MotionPlanner, UnsafeTrajectoryError,
                                             UnsafeJointPosition, ProbitedHandPosition)
from neu_ro_arm.robot.simulator_controller import SimulatorController
from neu_ro_arm.robot.xarm_controller import XArmController

class RobotArm:
    def __init__(self, controller_type='real'):
        '''Real or simulated xArm robot interface for safe, high-level motion commands

        Parameters
        ----------
        controller_type : str, {'real','sim'}, default to 'real'
            Indicate whether motor control be sent to a simulator or the real robot

        Attrbutes
        ---------
        joint_names : :tuple: str
            names of the joints in the arm
        controller : BaseController
            Controller used to execute motion commands
        mp : MotionPlanner
            Internal simulator of robot used to perform IK and collision
            detection
        '''
        self.joint_names = ('base', 'shoulder','elbow', 'wrist','wristRotation')

        if controller_type == 'real':
            self.controller = XArmController()
        elif controller_type == 'sim':
            self.controller = SimulatorController()
        else:
            raise TypeError('invalid controller type')
        self.controller_type = controller_type

        self.mp = MotionPlanner()
        self._mirror_planner()

    def home(self):
        '''Moves to home arm positions
        '''
        self.move_arm_jpos(self.controller.arm_jpos_home)
        self._mirror_planner()

    def passive_mode(self):
        if self.controller_type == 'real':
            self.controller.power_off()
        else:
            print('WARNING: passive mode does not exist for simulated robot')

    def active_mode(self):
        if self.controller_type == 'real':
            self.controller.power_on()
            self.mp.mirror(arm_jpos=self.get_arm_jpos(),
                           gripper_state=self.get_gripper_state())
        else:
            print('WARNING: active mode does not exist for simulated robot')


    def add_camera(self, pose_mtx):
        '''Add camera object to motion planner's internal simulator so that it is
        included in collision checking

        Parameters
        ----------
        pose_mtx : ndarray
            pose matrix of camera in world coordinate frame; shape=(4,4);
            dtype=float
        '''
        self.mp.add_camera(pose_mtx)

        if self.controller_type == 'sim':
            self.controller.add_camera(pose_mtx)

    def get_arm_jpos(self):
        '''Get positions of the arm joints

        Returns
        -------
        ndarray
            joint angles in radians; shape=(5,); dtype=float
        '''
        arm_jpos = self.controller.read_command(self.controller.arm_joint_idxs)
        return arm_jpos

    def move_arm_jpos(self, jpos, verbose=True):
        '''Moves arm joints to specific positions

        Parameters
        ----------
        jpos : ndarray
            Desired joint angles in radians for each joint in the arm;
            shape=(5,); dtype=float
        verbose : bool
            Whether to print error messages in case of an issue

        Returns
        -------
        bool
            True if joint angles returned from IK were achieved
        '''
        try:
            self.mp.check_arm_trajectory(jpos)
        except UnsafeTrajectoryError as e:
            if verbose:
                print(f"[MOVE FAILED] Trajectory would result in collision"
                      f" of robot:{e.robot_link} and {e.other_body}.")
            return False

        self.controller.move_command(self.controller.arm_joint_idxs, jpos)
        success, achieved_jpos = self.controller.monitor(self.controller.arm_joint_idxs,
                                                         jpos)
        self.mp.mirror(arm_jpos=achieved_jpos)
        return success

    def get_hand_pose(self):
        return self.mp.get_hand_pose()

    def move_hand_to(self, pos, rot=None, verbose=True):
        '''Moves end effector to desired pose in world

        Parameters
        ----------
        pos : ndarray
            desired 3d position of end effector; shape=(3,); dtype=float
        rot : ndarray
            desired euler angles of end effector; shape=(3,); dtype=float
        verbose : bool
            Whether to print error messages in case of an issue

        Raises
        ------
        UnsafeJointPosition
            If desired pose results in joint configuration that is in collision
            with the world
        UnsafeTrajectoryError
            If trajectory to reach desired pose will cause a collision.  See
            move_arm_jpos for details

        Returns
        -------
        bool
            True if joint angles returned from IK were achieved
        '''
        try:
            jpos, data = self.mp.calculate_ik(pos, rot)
        except ProbitedHandPosition as e:
            if verbose:
                print(f"[MOVE FAILED] {e}")
            return False
        except UnsafeJointPosition as e:
            if verbose:
                print(f"[MOVE FAILED] Target configuration would result in collision"
                      f" of robot:{e.robot_link} and {e.other_body}.")
            return False

        return self.move_arm_jpos(jpos)

    def open_gripper(self):
        '''Opens gripper completely

        Returns
        -------
        bool
            True if desired gripper state was achieved
        '''
        return self.set_gripper_state(GRIPPER_OPENED)

    def close_gripper(self):
        '''Closes gripper completely

        Returns
        -------
        bool
            True if desired gripper state was achieved
        '''
        return self.set_gripper_state(GRIPPER_CLOSED)

    def set_gripper_state(self, state):
        '''Get state of gripper

        Parameters
        ----------
        state : float
            gripper state to move to; must be in range [0,1]

        Returns
        -------
        bool
            True if desired gripper state was achieved
        '''
        assert 0 <= state <= 1
        jpos = self.controller.gripper_state_to_jpos(state)

        self.controller.move_command(self.controller.gripper_joint_idxs, jpos)
        success, achieved_jpos = self.controller.monitor(self.controller.gripper_joint_idxs,
                                                         jpos)

        achieved_gripper_state = self.controller.gripper_jpos_to_state(achieved_jpos)
        self.mp.mirror(gripper_state=achieved_gripper_state)
        return success

    def get_gripper_state(self):
        '''Get state of gripper

        Returns
        -------
        float
            value in range [0,1] describing how close to open(1) or closed(0)
            the gripper is
        '''
        jpos = self.controller.read_command(self.controller.gripper_joint_idxs)
        jpos = np.mean(jpos)
        state = self.controller.gripper_jpos_to_state(jpos)
        return state

    def _mirror_planner(self):
        self.mp.mirror(arm_jpos=self.get_arm_jpos(),
                       gripper_state=self.get_gripper_state())
