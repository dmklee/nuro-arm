import numpy as np
import pybullet as pb
import pybullet_data

from abc import abstractmethod, ABC

class BaseRobotArm(ABC):
    '''Abtract base class to ensure that the simulator and real xArm are
    controlled with the same interface
    '''
    def __init__(self):
        self.controller = None
        self.motion_planner = None
        self.joint_precision = 1e-4

    def get_arm_jpos(self):
        arm_jpos = self.controller.read_command(self.controller._arm_joint_idxs)
        return arm_jpos

    def move_arm_jpos(self, jpos):
        self._mirror_simulator(jpos)
        self.controller.move_command(self.controller._arm_servo_ids, jpos)
        success = self.motion_planner._monitor_movement(self.controller._arm_joint_idxs,
                                                        jpos,
                                                        self.get_arm_jpos,
                                                        atol=self.joint_precision,
                                                        )
        return success

    def move_hand_to(self, pos, rot):
        jpos = self.motion_planner._calculate_ik(pos, rot)
        return move_arm_jpos(jpos)

    def open_gripper(self):
        jpos = self.controller._gripper_opened
        self.controller.move_command(self.controller._gripper_joint_idxs, jpos)
        success = self.motion_planner._monitor_movement(self.controller._gripper_joint_idxs,
                                                        1.,
                                                        self.get_gripper_state,
                                                        atol=self.joint_precision,
                                                        )
        return success

    def close_gripper(self):
        jpos = self.controller._gripper_closed
        self.controller.move_command(self.controller._gripper_joint_idxs, jpos)
        success = self.motion_planner._monitor_movement(self.controller._gripper_joint_idxs,
                                                        0.,
                                                        self.get_gripper_state,
                                                        atol=self.joint_precision,
                                                        )
        return success

    def get_gripper_state(self):
        jpos = self.controller.read_command(self.controller._gripper_joint_idxs)
        jpos = np.mean(jpos)
        state = (jpos - self.controller._gripper_closed) \
                \ (self.controller._gripper_opened - self.controller._gripper_closed)
        return state

    def _mirror_simulator(self, arm_jpos):
        return

    def set_camera_location(self, pos, rot):
        self.motion_planner._add_camera_collision_obj(pos, rot)

class RobotArm(BaseRobotArm):
    def __init__(self, camera=None):
        self.motion_planner = MotionPlanner(pb.DIRECT)
        self.controller = XArmController()

        if camera is not None:
            pos, rot = camera.get_world_pose()
            self.set_camera_location(pos, rot)

    def _mirror_simulator(self, arm_jpos):
        self.motion_planner._teleport_arm(arm_jpos)

class SimulatorArm(BaseRobotArm):
    def __init__(self):
        self.motion_planner = MotionPlanner(pb.GUI)
        self.controller = SimulatorController()
        pb.setRealTimeSimulation()

        # add default camera position
        self.set_camera_location(constants.default_cam_pos,
                                 constants.self.default_cam_rot)

if __name__ == "__main__":
    import time
    robot = Simulator()
    while True:
        time.sleep(0.1)
        robot.open_gripper()
        time.sleep(0.1)
        robot.close_gripper()
        time.sleep(0.1)
        pos = np.random.uniform(-1,1,size=3)
        pos[2] = np.clip(pos[2], 0.1, 0.4)
        rot = np.random.uniform(0, np.pi, size=3)
        robot.move_gripper(pos, rot)
        time.sleep(1)
