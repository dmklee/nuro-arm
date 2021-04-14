from abc import ABCMeta, abstractmethod

class BaseController(metaclass=ABCMeta):
    gripper_opened = None
    gripper_closed = None
    arm_joint_idxs = None
    gripper_joint_idxs = None
    @abstractmethod
    def move_command(self, j_idxs, jpos):
        return

    @abstractmethod
    def read_command(self, j_idxs):
        return jpos

    def calc_gripper_state(self, gripper_jpos):
        return (gripper_jpos - self.gripper_closed[0]) \
                / (self.gripper_opened[0] - self.gripper_closed[0])

