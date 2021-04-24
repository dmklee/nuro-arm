import numpy as np
import pybullet as pb
import pybullet_data

import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
import tkinter as tk

from neu_ro_arm.constants import GRIPPER_CLOSED, GRIPPER_OPENED
from neu_ro_arm.robot.motion_planner import MotionPlanner, UnsafeTrajectoryError, UnsafeJointPosition
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

    def move_arm_jpos(self, jpos):
        '''Moves arm joints to specific positions

        Parameters
        ----------
        jpos : ndarray
            Desired joint angles in radians for each joint in the arm;
            shape=(5,); dtype=float

        Raises
        ------
        UnsafeTrajectoryError
            If trajectory to reach desired pose will cause a collision.

        Returns
        -------
        bool
            True if joint angles returned from IK were achieved
        '''
        safe, collision_data = self.mp.check_arm_trajectory(jpos)
        if not safe:
            raise UnsafeTrajectoryError(**collision_data)

        self.controller.move_command(self.controller.arm_joint_idxs, jpos)
        success, achieved_jpos = self.controller.monitor(self.controller.arm_joint_idxs,
                                                         jpos,
                                                        )
        self.mp.mirror(arm_jpos=achieved_jpos)
        return success

    def move_hand_to(self, pos, rot=None):
        '''Moves end effector to desired pose in world

        Parameters
        ----------
        pos : ndarray
            desired 3d position of end effector; shape=(3,); dtype=float
        rot : ndarray
            desired euler angles of end effector; shape=(3,); dtype=float

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
        is_safe, jpos, data = self.mp._calculate_ik(pos, rot)
        if not is_safe:
            raise UnsafeJointPosition(**data)

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
                                                         jpos,
                                                        )
        self.mp.mirror(gripper_jpos=achieved_jpos)
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

    def move_with_gui(self):
        '''Use interface to control positions of robot's joints

        Gui does not implement collision checking at the moment
        '''
        def move_joint_fn_generator(j_idx):
            def move_joint_fn(jpos):
                jpos = float(jpos)
                self.controller.move_command([j_idx], [jpos])
            return move_joint_fn

        def move_gripper_fn(state):
            state = float(state)
            gripper_jpos = self.controller.gripper_state_to_jpos(state)
            self.controller.move_command(self.controller.gripper_joint_idxs,
                                         gripper_jpos)

        def go_home_fn():
            self.controller.home()

        H,W = 500, 300
        window = tk.Tk()
        heading = tk.Label(text="BE CAREFUL!\nCollision detection is not running.",
                          fg="#FF0000")
        heading.pack()

        main_frame = tk.Frame(master=window, width=W, height=H)
        main_frame.pack(fill=tk.BOTH)

        # add scales for arm joints
        scales = []
        for j_idx, j_name in zip(self.controller.arm_joint_idxs, self.joint_names):
            row_frame = tk.Frame(master=main_frame, width=W,
                                 height=H//7, borderwidth=1)
            row_frame.pack(fill=tk.X)
            row_frame.pack_propagate(0)

            col_frame_left = tk.Frame(master=row_frame, width=W//3)
            col_frame_left.pack(side=tk.LEFT)

            col_frame_right = tk.Frame(master=row_frame, width=2*W//3)
            col_frame_right.pack(side=tk.RIGHT)

            lbl_joint = tk.Label(master=col_frame_left, text=j_name)
            lbl_joint.pack()

            move_joint_fn = move_joint_fn_generator(j_idx)
            scl_joint = tk.Scale(master=col_frame_right,
                                 from_=self.controller.joint_limits[j_idx][0],
                                 to=self.controller.joint_limits[j_idx][1],
                                 resolution=self.controller.joint_precision,
                                 orient=tk.HORIZONTAL,
                                 length=2*W//3,
                                 command=move_joint_fn)
            scl_joint.pack()
            scl_joint.pack_propagate(0)
            scales.append(scl_joint)

        arm_jpos = self.get_arm_jpos()
        [scl.set(jp) for scl, jp in zip(scales, arm_jpos)]

        #gripper
        row_frame = tk.Frame(master=main_frame, width=W,
                             height=H//7, borderwidth=1)
        row_frame.pack(fill=tk.X)
        row_frame.pack_propagate(0)
        col_frame_left = tk.Frame(master=row_frame, width=W//3)
        col_frame_left.pack(side=tk.LEFT)
        col_frame_right = tk.Frame(master=row_frame, width=2*W//3)
        col_frame_right.pack(side=tk.RIGHT)

        tk.Label(master=col_frame_left, text="gripper").pack()

        scl_gripper = tk.Scale(master=col_frame_right,
                             from_=0,
                             to=1,
                             resolution=0.1,
                             orient=tk.HORIZONTAL,
                             length=2*W//3,
                             command=move_gripper_fn)
        scl_gripper.pack()
        scl_gripper.pack_propagate(0)
        scl_gripper.set(self.get_gripper_state())

        # home button
        row_frame = tk.Frame(master=main_frame, width=W,
                             height=H//7, borderwidth=1)
        row_frame.pack(fill=tk.X)
        btn_go_home = tk.Button(row_frame, text="Go to HOME position",
                                fg="#0000FF", command = go_home_fn)
        btn_go_home.pack()

        window.mainloop()

if __name__ == "__main__":
    import time
    robot = RobotArm('sim')
    robot.controller.gripper_closed = np.array([0.5])
    robot.controller.gripper_opened = np.array([-0.5])
    robot.controller.home()
    robot.move_with_gui()
    # robot.controller.move_command([1],[0.2])
    # robot.controller.move_command([1],[0.24])
    # robot.controller.move_command([1],[0.28])
    # robot.controller.move_command([1],[0.29])
    # robot.controller.move_command([1],[0.27])
    # while True:
        # time.sleep(0.1)
    # robot.move_with_gui()
    # while True:
        # time.sleep(0.1)
        # robot.open_gripper()
        # time.sleep(0.1)
        # robot.close_gripper()
        # time.sleep(0.1)
        # pos = np.random.uniform(-1,1,size=3)
        # pos[2] = np.clip(pos[2], 0.1, 0.4)
        # rot = np.random.uniform(0, np.pi, size=3)
        # robot.move_hand_to(pos, rot)
        # time.sleep(1)
