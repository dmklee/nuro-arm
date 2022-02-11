import numpy as np
import pybullet as pb
import time

from nuro_arm.robot.base_controller import BaseController

class SimulatorController(BaseController):
    def __init__(self,
                 pb_sim,
                 realtime=True,
                ):
        BaseController.__init__(self)

        self.pb_sim = pb_sim
        self._unpack_simulator_params()
        self.arm_jpos_home = np.zeros(len(self.arm_joint_ids))

        pb.setGravity(0,0,-10,self._client)
        self.realtime = realtime

        self.position_gain = 0.2

    def timestep(self):
        [pb.stepSimulation() for _ in range(int(240/self.measurement_frequency))]
        if self.realtime:
            time.sleep(1./self.measurement_frequency)

    def power_on_servos(self):
        '''Turn on all servos so all joints are rigid
        '''
        joint_ids = self.arm_joint_ids + self.gripper_joint_ids
        current_jpos = self.read_jpos(joint_ids)
        pb.setJointMotorControlArray(self.robot_id,
                                     joint_ids,
                                     pb.POSITION_CONTROL,
                                     current_jpos,
                                     positionGains=len(joint_ids)*[self.position_gain]
                                    )

    def power_off_servos(self):
        '''Turn off all servos so all joints are passive
        '''
        joint_ids = self.arm_joint_ids+self.gripper_joint_ids
        pb.setJointMotorControlArray(self.robot_id,
                                     joint_ids,
                                     pb.POSITION_CONTROL,
                                     forces=len(joint_ids)*[0],
                                    )

    def power_on_servo(self, joint_id):
        '''Turn on single servo so the joint is rigid
        '''
        current_jpos = self.read_jpos([joint_id])[0]
        pb.setJointMotorControl2(self.robot_id,
                                 joint_id,
                                 pb.POSITION_CONTROL,
                                 current_jpos,
                                 positionGain=self.position_gain)

    def power_off_servo(self, joint_id):
        '''Turn off single servo so the joint is passive
        '''
        pb.setJointMotorControl2(self.robot_id,
                                 joint_id,
                                 pb.POSITION_CONTROL,
                                 force=0)

    def get_joint_id(self, joint_name):
        return {'base' : 1,
                'shoulder' : 2,
                'elbow' : 3,
                'wrist' : 4,
                'wristRotation' : 5,
                'gripper' : (7,10)}[joint_name]

    def write_jpos(self, joint_ids, jpos, speed=None):
        '''Issue move command to specified joint indices

        Parameters
        ----------
        jpos : array_like of float
            target joint positions corresponding to the joint indices
        speed : float, array_like of float
            designate movement speed.

        Returns
        -------
        float
            expected time (s) to complete movement, used for monitoring
        '''
        if speed is None:
            speed = self.default_speed
        if np.isscalar(speed):
            speed = np.full(len(joint_ids), speed)

        current_jpos = self.read_jpos(joint_ids)
        duration = np.abs(np.subtract(current_jpos, jpos))/speed

        # in pybullet==3.17, maxVelocity is not exposed in setJointMotorControlArray
        # so we have to send commands individually
        for i in range(len(joint_ids)):
            pb.setJointMotorControl2(self.robot_id,
                                     joint_ids[i],
                                     pb.POSITION_CONTROL,
                                     jpos[i],
                                     positionGain=self.position_gain,
                                     maxVelocity=speed[i],
                                     physicsClientId=self._client
                                    )
        return np.max(duration)

    def read_jpos(self, joint_ids):
        '''Read current joint positions

        Parameters
        ----------
        joint_ids : array_like of int
            indices of joints

        Returns
        -------
        array_like of float
            joint positions in radians in same order as joint_ids
        '''
        jpos = next(zip(*pb.getJointStates(self.robot_id,
                                           joint_ids,
                                           physicsClientId=self._client
                                          )))
        return jpos

    def _unpack_simulator_params(self):
        self.robot_id = self.pb_sim.robot_id
        self._client = self.pb_sim._client
        self.n_joints = self.pb_sim.n_joints
        self.arm_joint_ids = self.pb_sim.arm_joint_ids
        self.arm_joint_limits = self.pb_sim.arm_joint_limits
        self.gripper_joint_ids = self.pb_sim.gripper_joint_ids
        self.gripper_joint_limits = self.pb_sim.gripper_joint_limits
        self.link_names = self.pb_sim.link_names
        self.end_effector_link_index = self.pb_sim.end_effector_link_index

