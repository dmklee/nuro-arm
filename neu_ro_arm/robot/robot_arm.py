import numpy as np
import pybullet as pb
import pybullet_data

from abc import abstractmethod, ABC
import constants as constants
from robot.motion_planner import MotionPlanner, UnsafeTrajectoryError
from robot.simulator_controller import SimulatorController
from robot.xarm_controller import XArmController

class RobotArm(ABC):
    '''Abtract base class to ensure that the simulator and real xArm are
    controlled with the same interface
    '''
    def __init__(self, controller_type):
        self.joint_names = ('base', 'shoulder','elbow', 'wrist','wristRotation')

        if controller_type == 'real':
            self.controller = XArmController
        elif controller_type == 'sim':
            self.controller = SimulatorController
        else:
            raise TypeError('invalid controller type')

        self.mp = MotionPlanner()

    def add_camera(self, camera):
        '''This must be incorporated for collision checking'''
        camera2world = camera.camera2world
        self.mp.add_camera(world2camera)

    def get_arm_jpos(self):
        arm_jpos = self.controller.read_command(self.controller.arm_joint_idxs)
        return arm_jpos

    def move_arm_jpos(self, jpos):
        is_collision = self.mp.check_arm_trajectory(jpos)
        if is_collision:
            raise UnsafeTrajectoryError

        self.controller.move_command(self.controller.arm_joint_idxs, jpos)
        success, achieved_jpos = self.controller.monitor(self.controller.arm_joint_idxs,
                                                         jpos,
                                                        )
        self.mp.mirror(arm_jpos=achieved_jpos)
        return success

    def move_hand_to(self, pos, rot):
        is_collision, jpos = self.mp._calculate_ik(pos, rot)
        if is_collision:
            raise UnsafeJointPosition

        return self.move_arm_jpos(jpos)

    def open_gripper(self):
        return self.set_gripper_state(self.gripper_opened)

    def close_gripper(self):
        return self.set_gripper_state(self.gripper_closed)

    def set_gripper_state(self, state):
        assert 0 <= state <= 1
        jpos = self.controller.gripper_state_to_jpos(state)

        self.controller.move_command(self.controller.gripper_joint_idxs, jpos)
        success, achieved_jpos = self.controller.monitor(self.controller.gripper_joint_idxs,
                                                         jpos,
                                                        )
        self.mp.mirror(gripper_jpos=achieved_jpos)
        return success

    def get_gripper_state(self):
        jpos = self.controller.read_command(self.controller.gripper_joint_idxs)
        jpos = np.mean(jpos)
        state = self.controller.gripper_jpos_to_state(jpos)
        return state

    def move_with_gui(self):
        def move_joint_fn_generator(j_idx):
            def move_joint_fn(jpos):
                self.controller.move_command([j_idx], [jpos])
            return move_joint_fn

        H,W = 600, 400
        import tkinter as tk
        window = tk.Tk()
        heading = tk.Label(text="GUI")
        heading.pack()

        main_frame = tk.Frame(master=window, width=W, height=H)
        main_frame.pack(fill=tk.BOTH)

        # add scales for arm joints
        scales = []
        for j_idx, j_name in zip(self.controller.arm_joint_idxs, self.joint_names):
            row_frame = tk.Frame(master=main_frame, width=W,
                                 height=H//7, borderwidth=1)
            row_frame.pack(fill=tk.X)

            col_frame_left = tk.Frame(master=row_frame, width=W//2)
            col_frame_left.pack(side=tk.LEFT)
            col_frame_right = tk.Frame(master=row_frame, width=W//2)
            col_frame_right.pack(side=tk.RIGHT)

            lbl_joint = tk.Label(master=col_frame_left, text=j_name)
            lbl_joint.pack()

            move_joint_fn = move_joint_fn_generator(j_idx)
            scl_joint = tk.Scale(master=col_frame_right,
                                 from_=self.controller.joint_limits[j_idx, 0],
                                 to=self.controller.joint_limits[j_idxs,1],
                                 resolution=self.controller.joint_precision,
                                 orient=tk.HORIZONTAL,
                                 command=move_joint_fn)
            scl_joint.pack()
            scales.append(scl_joint)

        arm_jpos = self.get_arm_jpos()
        [scl.set(jp) for scl, jp in zip(scales, arm_jpos)]

        #TODO: add scale for gripper state

        window.main_loop()

class RobotArm(BaseRobotArm):
    def __init__(self, camera=None):
        self.mp = MotionPlanner(pb.GUI)
        self.controller = XArmController()

        if camera is not None:
            pos, rot = camera.get_world_pose()
            self.set_camera_location(pos, rot)

    def _mirror_simulator(self, arm_jpos):
        self.mp._teleport_arm(arm_jpos)

class SimulatorArm(BaseRobotArm):
    def __init__(self):
        self.mp = MotionPlanner(pb.GUI)
        self.controller = SimulatorController(self.mp.robot_id)
        pb.setRealTimeSimulation(1)

        # add default camera position
        self.set_camera_location(constants.default_cam_pos,
                                 constants.default_cam_rot)

if __name__ == "__main__":
    import time
    robot = SimulatorArm()
    while True:
        time.sleep(0.1)
        robot.open_gripper()
        time.sleep(0.1)
        robot.close_gripper()
        time.sleep(0.1)
        pos = np.random.uniform(-1,1,size=3)
        pos[2] = np.clip(pos[2], 0.1, 0.4)
        rot = np.random.uniform(0, np.pi, size=3)
        robot.move_hand_to(pos, rot)
        time.sleep(1)
