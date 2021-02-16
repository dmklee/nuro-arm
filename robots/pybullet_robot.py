import numpy as np
import os

import pybullet as pb
import pybullet_data
import serial
from ikpy.chain import Chain
from abc import abstractmethod

def toDeg(angle):
    '''Convert radians to degrees'''
    return 180. * angle / np.pi

def toRad(angle):
    '''Convert degrees to radians'''
    return np.pi * angle / 180.

HOME = (90,90,90,90,90,73)

class RobotSimulator:
    def __init__(self, render=False):
        self.home_joint_states = toRad(np.array((0, 90,90,90,90,90,73,73)))
        self.arm_joint_indices = (1,2,3,4,5)
        self.gripper_joint_indices = (6,7)
        self.gripper_joint_limits = (0,0.3)

        self.urdf_filepath = 'ros_braccio_urdf/urdf/braccio_arm.urdf'

        if simulated:
            self.client = pb.connect(pb.GUI)
        else:
            self.client = pb.connect(pb.DIRECT)
        self.sim_id = pb.loadURDF(self.urdf_filepath, [0,0,0], [0,0,0,1])
        [pb.resetJointState(self.sim_id, idx, state) for idx,state in
         enumerate(self.home_joint_states)]

    def _calculate_IK(self, pos, rot):
        '''
        pos: vec3 <x,y,z>
        rot: vec4 quaternion <x,y,z,w>
        '''
        return pb.calculateInverseKinematics(self.sim_id,
                                             self.gripper_joint_indices[0],
                                             pos, rot)

    @abstractmethod
    def go_home(self):
        raise NotImplementedError

    def open_gripper(self):
        ''''''
        p1, p2 = self._get_gripper_joint_states()
        self._send_gripper_command(self.gripper_joint_limits[1])
        it = 0
        while not np.isclose(p1, self.gripper_joint_limits[1]):
            pb.stepSimulation()
            it += 1
            if it > 100:
                return False
            p1, p2 = self._get_gripper_joint_states()
        return True

    def close_gripper(self):
        ''''''
        p1, p2 = self._get_gripper_joint_states()
        limit = self.gripper_joint_limits[0]
        self._send_gripper_command(limit)
        # self._sendGripperCloseCommand()
        it = 0
        while not (np.isclose(p1, self.gripper_joint_limits[1]) and
                   np.isclose(p2, self.gripper_joint_limits[1])):
            pb.stepSimulation()
            it += 1
            p1_, p2_ = self._get_gripper_joint_states()
            if it > 100 or (abs(p1-p1_)<0.0001 and abs(p2-p2_)<0.0001):
                mean = (p1+p2)/2 + 0.01
                self._send_gripper_command(mean)
                return False
            p1 = p1_
            p2 = p2_
        return True

    def _get_arm_joint_angles(self):
        return list(zip(*pb.getJointStates(self.sim_id, self.arm_joint_indices)))[0]

    def _get_gripper_joint_states(self):
        return (np.pi/2 - pb.getJointState(self.sim_id, self.gripper_joint_indices[0])[0],
                pb.getJointState(self.sim_id, self.gripper_joint_indices[1])[0] - np.pi/2)

    @abstractmethod
    def move_to(self, position):
        '''move to end effector position'''
        raise NotImplementedError

    def _send_gripper_command(self, target_pos):
        pb.setJointMotorControlArray(
            self.id, self.gripper_joint_indices, pb.POSITION_CONTROL,
            targetPositions=[np.pi/2-target_pos, np.pi/2-target_pos],
            forces=self.max_forces[-2:],
            positionGains=[0.02]*2, velocityGains=[1.0]*2
        )

    def _send_arm_command(self, target_pos):
        num_servos = len(self.arm_joint_indices)
        pb.setJointMotorControlArray(
            self.id, self.arm_joint_indices, pb.POSITION_CONTROL,
            targetPositions=target_pos,
            forces=self.max_forces[:-2],
            positionGains=[0.02]*num_servos, velocityGains=[1.0]*num_servos
        )

class JointState:
    HOME = np.array((1.5708, 1.5708, 1.5708, 1.5708, 1.5708, 1.2740))
    URDF_PATH = 'ros_braccio_urdf/urdf/braccio_arm.urdf'
    def __init__(self):
        self.chain = Chain.from_urdf_file(self.URDF_PATH)
        self.angles = self.HOME.copy()
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
        self.angles[-1] = toRad(73)

    def close_gripper(self):
        self.angles[-1] = toRad(10)

    def set_gripper_position(self, position):
        '''sets angles to ik solution, returns actual achieved position'''
        self.angles = self.chain.inverse_kinematics(position)

    def set_angles(self, angles):
        self.angles = np.asarray(angles)
        self.enforce_limits()

    def get_end_effector_pose(self):
        # return 3D position, 
        pose = self.chain.forward_kinematics(self.angles)
        return pose_matrix[:3,3], pose_matrix[:3,:3]

    def enforce_limits(self):
        self.angles = np.clip(self.angles, *self.joint_limits.T)

    def __setitem__(self, key, newvalue):
        assert key in self.joint_names, 'incorrect joint name'
        joint_id = self.joint_names.index(key)
        self.angles[joint_id] = newvalue
        self.angles.enforce_limits()

    def __getitem__(self, key):
        assert key in self.joint_names, 'incorrect joint name'
        joint_id = self.joint_names.index(key)
        return self.angles[joint_id]

class ArduinoRobot:
    #TODO: reflect arduino robot state in simulator
    def __init__(self, port_name='/dev/ttyACM0', render=False):
        self.sim = RobotSimulator(render)
        self.port = serial.Serial(port_name, 115200, timeout=5)

        self.joint_names = ('base', 'shoulder', 'elbow', 'wrist',
                            'wristRotation', 'gripper')
        self.home_angles = toRad(np.array((90,90,90,90,90,73)))
        self.joint_angles = self.home_angles.copy()
        self.gripper_open = toRad(10)
        self.gripper_closed = toRad(73)

    def open_gripper(self):
        self.move_to_angles(self.joint_angles.open_gripper())

    def close_gripper(self):
        self.move_to_angles(self.joint_angles.close_gripper())

    def move_to_angles(self, joint_angles):
        pass

    def go_home(self):
        self._write_serial('H\n')

    def power_on(self):
        self._write_serial('1\n')

    def power_off(self):
        self._write_serial('0\n')

    def __del__(self):
        self.power_off()

    def _write_serial(self, string):
        self.port.write(string.encode())
        self.port.readline()

class Robot:
    def __init__(self, urdf_filepath, render=True):
        self.urdf_filepath = urdf_filepath
        self.home_joint_states = toRad(np.array((90,90,90,90,90,73,73)))
        self.max_forces = np.array((50,30,30,30,30,30,30))
        self.arm_joint_indices = (1,2,3,4,5)
        self.gripper_joint_indices = (6,7)
        self.gripper_joint_limits = (0,0.3)
        self.joint_names = ['base', 'shoulder', 'elbow', 'wristRotation',
                            'wrist', 'gripper_left', 'gripper_right']
        if render:
            self.client = pb.connect(pb.GUI)
        else:
            self.client = pb.connect(pb.DIRECT)
        self.initialize()
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        # pb.setGravity(0,0,-10)
        # self.plane_id = pb.loadURDF('plane.urdf', [0,0,0])

    def initialize(self):
        '''
        Load robot from urdf
        set joint angles to home
        '''
        self.id = pb.loadURDF(self.urdf_filepath, [0,0,0], [0,0,0,1])
        self.end_effector_id = 5
        [pb.resetJointState(self.id, idx, state) for idx,state in
         enumerate(self.home_joint_states)]


    # def move_arm(self, pos, rot):
        # self._calculate_IK(pos, rot)
        # self._send_arm_command(ik)
        # ls = pb.getLinkState(self.id, self.end_effector_id)
        # it = 0
        # while not np.allclose(target_pos - arm_state, atol=1e-2):
            # pb.stepSimulation()
            # it += 1
            # arm_state = self._get_arm_joint_states()
            # if it > 1000:
                # self._send_arm_command(arm_state)
                # return False
        # return True

    def move_arm_joints(self, target_angles):
        current_angles = self._get_arm_joint_angles()
        self._send_arm_command(target_angles)
        it = 0
        while not np.allclose(current_angles, target_angles, atol=1e-2):
            pb.stepSimulation()
            it += 1
            current_angles = self._get_arm_joint_angles()
            if it > 1000:
                self._send_arm_command(current_angles)
                return False
        return True

    def _send_arm_command(self, target_pos):
        num_servos = len(self.arm_joint_indices)
        pb.setJointMotorControlArray(
            self.id, self.arm_joint_indices, pb.POSITION_CONTROL,
            targetPositions=target_pos,
            forces=self.max_forces[:-2],
            positionGains=[0.02]*num_servos, velocityGains=[1.0]*num_servos
        )

    def _get_arm_joint_angles(self):
        return list(zip(*pb.getJointStates(self.id, self.arm_joint_indices)))[0]

    def _get_gripper_joint_states(self):
        return (np.pi/2 - pb.getJointState(self.id, self.gripper_joint_indices[0])[0],
                pb.getJointState(self.id, self.gripper_joint_indices[1])[0] - np.pi/2)

    def _end_effector_pose(self, arm_joint_states):
        pass

    def _calculate_IK(self, pos, rot):
        '''
        pos: vec3 <x,y,z>
        rot: vec4 quaternion <x,y,z,w>
        '''
        return pb.calculateInverseKinematics(self.id, self.end_effector_id,
                                             pos, rot)


if __name__ == "__main__":
    import time
    filepath = 'ros_braccio_urdf/urdf/braccio_arm.urdf'
    r = Robot(filepath)
    while True:
        position = np.random.uniform(-1,1,size=3)
        euler_angle = np.random.uniform(-np.pi,np.pi,size=3)
        quat = pb.getQuaternionFromEuler(euler_angle)
        ik = r._calculate_IK(position, quat)
        r.move_arm_joints(ik[:5])
        time.sleep(0.5)
