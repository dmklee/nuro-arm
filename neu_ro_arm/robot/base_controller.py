import numpy as np
import time

from abc import abstractmethod

class BaseController:
    def __init__(self):
        '''Base class for controller to implement move and read commands
        '''
        self.gripper_opened = None
        self.gripper_closed = None
        self.arm_joint_idxs = None
        self.arm_jpos_home = None
        self.gripper_joint_idxs = None
        self.joint_limits = None
        self.movement_precision = 1e-4
        self.measurement_precision = 1e-4
        self.measurement_frequency = 20

    @abstractmethod
    def move_command(self, j_idxs, jpos, speed=None):
        '''Issue move command to specified joint indices

        All movements are linear in joint space

        Parameters
        ----------
        j_idxs : array_like of int
            joint indices to be moved
        jpos : array_like of float
            target joint positions corresponding to the joint indices
        speed : {'normal', 'max', 'slow'}, optional
            designate movement speed. Not used by simulator controller

        Returns
        -------
        float
            expected time (s) to complete movement, used for monitoring
        '''
        return

    @abstractmethod
    def read_command(self, j_idxs):
        return

    def home(self):
        '''Move arm to home joint positions
        '''
        self.move_command(self.arm_joint_idxs,
                          self.arm_jpos_home)

    def gripper_jpos_to_state(self, jpos):
        '''Convert gripper joint position to state

        Parameters
        ----------
        jpos : array_like of float
            gripper joint positions

        Returns
        -------
        float
            gripper state
        '''
        jpos = np.mean(jpos)
        return (jpos - self.gripper_closed[0]) \
                / (self.gripper_opened[0] - self.gripper_closed[0])

    def gripper_state_to_jpos(self, state):
        '''Convert gripper state to gripper joint positions

        Parameters
        ----------
        state : float
            gripper state, should be in range from 0 to 1

        Returns
        -------
        array_like
            gripper joint position
        '''
        return state*self.gripper_opened + (1-state)*self.gripper_closed


    def monitor(self, j_idxs, target_jpos, duration=2):
        '''Monitor controller motion to detect failure or collision

        With simulated controller, failure indicates collision.  With xArm, it
        is possible that the joint precision is too small (the arm is quite
        inconsistent with hitting the target position especially for short
        movements).

        Parameters
        ----------
        j_idxs : array_like of int
            indices of joints to monitor, order should correspond to order of
            target_jpos
        target_jpos : array_like of float
            target joint positions in radians, length should match j_idxs
        duration : float, default 2
            number of seconds that the movement is expected to take. a movement
            will be labeled a failure if it takes longer than 1.1x duration

        Returns
        -------
        bool
            True if target joint position was reached
        array_like
            achieved joint position at the end of the motion, may be different
            from target even if successful due to joint precision margin
        '''
        t_factor = 1.1
        start_time = time.time()

        jpos = self.read_command(j_idxs)
        while not np.allclose(jpos, target_jpos, atol=self.movement_precision):
            time.sleep(1./self.measurement_frequency)

            new_jpos = self.read_command(j_idxs)
            if time.time()-start_time > t_factor * duration:
                # movement has taken too much time
                return False, jpos

            if (np.abs(np.subtract(new_jpos, jpos)) < self.measurement_precision).all():
                # all motion has stopped
                return False, jpos

            jpos = new_jpos

        return True, jpos

