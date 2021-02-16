import numpy as np
from ikpy.chain import Chain

class JointStates:
    HOME = np.array((1.5708, 1.5708, 1.5708, 1.5708, 1.5708, 1.2740))
    URDF_PATH = 'ros_braccio_urdf/urdf/braccio_arm.urdf'
    def __init__(self):
        self.chain = Chain.from_urdf_file(self.URDF_PATH)
        self._angles = self.HOME.copy()
        # taken from https://github.com/arduino-libraries/Braccio/blob/master/src/Braccio.cpp
        self.joint_limits = np.array(((0.      , 3.1416),
                                      (0.2618  , 2.8798),
                                      (0.      , 3.1416),
                                      (0.      , 3.1416),
                                      (0.      , 3.1416),
                                      (0.1745  , 1.2740)))
        self.joint_names = ('base', 'shoulder', 'elbow', 'wrist',
                            'wristRotation', 'gripper')

    def open_gripper(self):
        self._angles[-1] = self.joint_limits[-1,0]

    def close_gripper(self):
        self._angles[-1] = self.joint_limits[-1,1]

    def set_gripper_position(self, position):
        '''sets angles to ik solution, returns actual achieved position'''
        self._angles = self.chain.inverse_kinematics(position)

    def set_angles(self, angles):
        self._angles = np.asarray(angles)
        self.enforce_limits()

    def angles(self):
        return self._angles

    def get_gripper_position(self):
        # return 3D position, 
        pose_matrix = self.chain.forward_kinematics(list(self._angles) \
                                                    + [self._angles[-1]])
        return pose_matrix[:3,3]

    def enforce_limits(self):
        self._angles = np.clip(self._angles, *self.joint_limits.T)

    def __setitem__(self, key, newvalue):
        assert key in self.joint_names, 'incorrect joint name'
        joint_id = self.joint_names.index(key)
        self._angles[joint_id] = newvalue
        self._angles.enforce_limits()

    def __getitem__(self, key):
        assert key in self.joint_names, 'incorrect joint name'
        joint_id = self.joint_names.index(key)
        return self._angles[joint_id]

    def to_degrees(self, round=False):
        degrees = 180 * self._angles / np.pi
        return degrees.astype(int) if round else degrees

    def __len__(self):
        return len(self._angles)

if __name__ == "__main__":
    js = JointStates()
    print(js.get_gripper_position())
