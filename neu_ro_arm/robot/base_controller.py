import numpy as np
import time

from abc import abstractmethod

class BaseController:
    gripper_opened = None
    gripper_closed = None
    arm_joint_idxs = None
    gripper_joint_idxs = None
    joint_limits = None
    joint_precision = 1e-4

    @abstractmethod
    def move_command(self, j_idxs, jpos):
        return

    @abstractmethod
    def read_command(self, j_idxs):
        return jpos

    def gripper_jpos_to_state(self, jpos):
        return (jpos - self.gripper_closed[0]) \
                / (self.gripper_opened[0] - self.gripper_closed[0])

    def gripper_state_to_jpos(self, state):
        return state*self.gripper_opened + (1-gripper_state)*self.gripper_closed

    def monitor(self, j_idxs, target_jpos, max_iter=100, monitor_freq=10):
        it = 0

        jpos = self.read_command(j_idxs)
        while not np.allclose(old_jpos, target_jpos, atol=self.joint_precision):
            it += 1
            time.sleep(1./monitor_freq)

            new_jpos = self.read_command(j_idxs)
            if it > max_iter or np.allclose(new_jpos, jpos):
                # motion has stopped, so failure
                return False, jpos

            jpos = new_jpos

        return True, jpos

