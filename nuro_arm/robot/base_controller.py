import numpy as np
import time

from abc import abstractmethod

class BaseController:
    def __init__(self):
        '''Base class for controller to implement move and read commands
        '''
        self.joint_names = ('base', 'shoulder','elbow',
                            'wrist','wristRotation', 'gripper')

        self.arm_joint_ids = None
        self.gripper_joint_ids = None

        self.arm_joint_limits = None
        self.gripper_joint_limits = None

        self.arm_jpos_home = None

        self.movement_precision = 1e-4
        self.measurement_precision = 1e-4
        self.measurement_frequency = 10

        self.default_speed = 0.8

    def read_arm_jpos(self):
        '''Get current joint positions of arm servos

        Returns
        -------
        array_like
            joint positions in radians, ordered from base to wristRotation
        '''
        return self.read_jpos(self.arm_joint_ids)

    def read_gripper_state(self):
        '''Get current gripper state

        Returns
        -------
        float
            gripper state from 0 (fully closed) to 1 (fully opened)
        '''
        gripper_jpos = self.read_jpos(self.gripper_joint_ids)
        gripper_state = self._gripper_jpos_to_state(gripper_jpos)
        gripper_state = np.clip(gripper_state, 0, 1)
        return gripper_state

    def write_arm_jpos(self, jpos, speed=None):
        '''Send movement command to arm servos

        Parameters
        ----------
        jpos : array_like
            target arm joint positions in radians, ordered from base to
            wristRotation
        speed : float, array_like of float, default to None
            movement speed in radians per second.  by providing array_like,
            speed for each servo can be specified

        Returns
        -------
        float
            expected time (s) to complete movement, used for monitoring
        '''
        return self.write_jpos(self.arm_joint_ids, jpos, speed)

    def write_gripper_state(self, state, speed=None):
        '''Send movement command to achieve specified gripper state

        Parameters
        ----------
        jpos : float
            target gripper state in range 0 to 1
        speed : float
            movement speed in radians per second

        Returns
        -------
        float
            expected time (s) to complete movement, used for monitoring
        '''
        state = np.clip(state, 0, 1)
        gripper_jpos = self._gripper_state_to_jpos(state)
        return self.write_jpos(self.gripper_joint_ids, gripper_jpos, speed)

    @abstractmethod
    def timestep(self):
        '''Used for monitoring movements, either waits some time or steps simulator
        if applicable
        '''
        pass

    @abstractmethod
    def get_joint_id(self, joint_name):
        '''Get joint id associated with a given joint name

        Parameters
        ----------
        joint_name : str, {'base', 'shoulder','elbow','wrist','wristRotation','gripper'}

        Returns
        -------
        int
            joint id
        '''
        pass

    @abstractmethod
    def get_joint_name(self, joint_id):
        '''Get joint name associated with a given joint id

        Parameters
        ----------
        joint_id : int

        Returns
        -------
        str
            joint name
        '''
        pass

    @abstractmethod
    def power_on_servos(self):
        '''Turn on all servos so all joints are rigid
        '''
        return

    @abstractmethod
    def power_off_servos(self):
        '''Turn off all servos so all joints are passive
        '''
        return

    @abstractmethod
    def power_on_servo(self, joint_id):
        '''Turn on single servo so the joint is rigid

        Parameters
        ----------
        joint_id : int
        '''
        pass

    @abstractmethod
    def power_off_servo(self, joint_id):
        '''Turn off single servo so the joint is passive

        Parameters
        ----------
        joint_id : int
        '''
        pass

    @abstractmethod
    def read_jpos(self, joint_ids):
        '''Read joint positions

        Parameters
        ----------
        joint_ids : array_like of int
        '''
        pass

    @abstractmethod
    def write_jpos(self, joint_ids, jpos, speed):
        '''Issue move command

        Parameters
        ----------
        joint_ids : array_like of int
            joint indices
        jpos : array_like of float
            target joint positions corresponding to the joint indices
        speed : float, array_like of float
            designate movement speed.

        Returns
        -------
        float
            expected time (s) to complete movement, used for monitoring
        '''
        pass

    def monitor(self, joint_ids, target_jpos, expected_duration):
        '''Monitor arm movement to detect failure or collision

        With simulated controller, failure indicates collision.  With xArm, it
        is also possible that the joint precision is too small (I have tried to
        tune the precision values, however the gripper is especially inaccurate
        for some reason).

        Parameters
        ----------
        j_idxs : array_like of int
            indices of joints to monitor, order should correspond to order of
            target_jpos
        target_jpos : array_like of float
            target joint positions in radians, length should match j_idxs
        duration : float
            number of seconds that the movement is expected to take. a movement
            will be labeled a failure if it takes longer than 1.5x duration

        Returns
        -------
        bool
            True if target joint position was reached
        array_like
            achieved joint position at the end of the motion, may be different
            from target even if successful due to joint precision margin
        '''
        t_factor = 1.5
        start_time = time.time()

        jpos = self.read_jpos(joint_ids)

        # give some initial time for motion to start
        # otherwise, it may terminate prematurely because it detects no motion
        # this is mainly an issue for the gripper only
        [self.timestep() for _ in range(4)]
        while True:
            self.timestep()

            new_jpos = self.read_jpos(joint_ids)
            if np.allclose(new_jpos, target_jpos, atol=self.movement_precision):
                # success
                return True, np.array(new_jpos)

            if time.time()-start_time > t_factor * expected_duration:
                # movement has taken too much time
                return False, np.array(new_jpos)

            if (np.abs(np.subtract(new_jpos, jpos)) < self.measurement_precision).all():
                # all movements are within measurement precision so motion has stopped
                return False, np.array(new_jpos)

            jpos = new_jpos

    def _gripper_jpos_to_state(self, jpos):
        '''Convert gripper joint position to state

        Parameters
        ----------
        jpos : array_like of float
            gripper joint position

        Returns
        -------
        float
            gripper state
        '''
        state = (jpos - self.gripper_joint_limits[0]) \
                / (self.gripper_joint_limits[1] - self.gripper_joint_limits[0])
        return np.mean(state)

    def _gripper_state_to_jpos(self, state):
        '''Convert gripper state to gripper joint positions

        Parameters
        ----------
        state : float
            gripper state, should be in range from 0 to 1

        Returns
        -------
        array_like of float
            gripper joint position
        '''
        state = np.clip(state, 0, 1)
        return (1-state)*self.gripper_joint_limits[0] + state*self.gripper_joint_limits[1]
