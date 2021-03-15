import numpy as np
from ikpy.chain import Chain

class JointStates:
    def __init__(self, home, joint_limits, urdf_path):
        self.chain = Chain.from_urdf_file(urdf_path)
        self.home = home
        self._angles = self.home.copy()
        # taken from https://github.com/arduino-libraries/Braccio/blob/master/src/Braccio.cpp
        self.joint_limits = joint_limits
        self.joint_names = {'base':0,
                            'shoulder': 1,
                            'elbow' : 2,
                            'wrist' : 3,
                            'wristRotation' : 4,
                            'gripper' : (5,6)}

    def open_gripper(self):
        self._angles[-1] = self.joint_limits[-1,0]

    def close_gripper(self):
        self._angles[-2:] = self.joint_limits[-1,1]

    def set_gripper_position(self, position):
        '''sets angles to ik solution, returns actual achieved position'''
        self._angles = self.chain.inverse_kinematics(position)

    def set_angles(self, angles):
        self._angles = np.asarray(angles)
        self.enforce_limits()

    def angles(self):
        # the last two joints are the same, so ignore
        return self._angles[:-1]

    def get_gripper_position(self):
        # return 3D position, 
        pose_matrix = self.chain.forward_kinematics(self._angles)
        return pose_matrix[:3,3]

    def enforce_limits(self):
        self._angles = np.clip(self._angles, *self.joint_limits.T)

    def __setitem__(self, key, newvalue):
        assert key in self.joint_names, 'incorrect joint name'
        joint_id = self.joint_names[key]
        self._angles[joint_id] = newvalue
        self.enforce_limits()

    def __getitem__(self, key):
        assert key in self.joint_names, 'incorrect joint name'
        joint_id = self.joint_names[key]
        if isinstance(joint_id, tuple): joint_id = joint_id[0]
        return self._angles[joint_id]

    def to_degrees(self, round=False):
        degrees = 180 * self._angles / np.pi
        return degrees.astype(int) if round else degrees

    def __len__(self):
        return len(self._angles)

if __name__ == "__main__":
    js = JointStates()
    print(js.get_gripper_position())
