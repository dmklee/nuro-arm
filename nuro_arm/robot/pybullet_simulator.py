import pybullet_data
import pybullet as pb
import numpy as np
import os

import nuro_arm
from nuro_arm import transformation_utils, constants

class PybulletSimulator:
    def __init__(self,
                 headless=True,
                 client=None,
                ):
        '''Base class for pybullet simulator to handle initialization and attributes
        about robot joints

        Parameters
        ----------
        connection_mode : {pb.GUI, pb.DIRECT}
            Indicates whether pybullet simulator should generate GUI or not

        Attributes
        ----------
        end_effector_link_index : int
            index of hand link, this is used to specify which link
            should be used for IK
        arm_joint_ids: list of int
            indices of joints that control the arm
        gripper_joint_ids: list of int
            indices of two gripper joints. for simulator, gripper operation is
            controlled by two joints which should both move in concert to reflect
            the real xArm
        gripper_closed : ndarray
            joint positions of gripper joints that result in closed gripper in
            the simulator; shape = (2,); dtype=float
        gripper_opened : ndarray
            joint positions of gripper joints that result in opened gripper in
            the simulator; shape = (2,); dtype=float
        '''
        self.arm_joint_ids = [1,2,3,4,5]
        self.gripper_joint_ids = [6,7]
        self.dummy_joint_ids = [8]
        self.finger_joint_ids = [9,10]
        self.end_effector_link_index = 11

        self.arm_joint_limits = np.array(((-2, -1.58, -2, -1.8, -2),
                                          ( 2,  1.58,  2,  2.0,  2)))

        self.gripper_joint_limits = np.array(((0.05,0.05),
                                              (1.38, 1.38)))
        self.dummy_joint_limits = np.array(((0.025,),(0.055,)))
        self.finger_joint_limits = np.array(((0.0145, 0.029,),
                                             (0.0445, 0.089,)))

        connection_mode = pb.DIRECT if headless else pb.GUI
        if client is None:
            self._client = self._initialize_client(connection_mode)
        else:
            self._client = client

        # suction cups are 12 mm tall when not pressed
        robot_pos = (0, 0, 0.012)
        robot_rot = (0, 0, 0, 1)
        self.robot_id = self.initialize_robot(robot_pos, robot_rot)
        self.n_joints = pb.getNumJoints(self.robot_id,
                                          physicsClientId=self._client)

        self.joint_names = []
        self.link_names = []
        self.joint_ll = []
        self.joint_ul = []
        for j_id in range(self.n_joints):
            j_info = pb.getJointInfo(self.robot_id, j_id, physicsClientId=self._client)
            self.joint_names.append(j_info[1].decode('ascii'))
            self.joint_ll.append(j_info[8])
            self.joint_ul.append(j_info[9])
            self.link_names.append(j_info[12].decode('ascii'))
        self.joint_ll = np.array(self.joint_ll)
        self.joint_ul = np.array(self.joint_ul)

    def _initialize_client(self, connection_mode):
        '''Creates pybullet simulator and loads world plane.

        Parameters
        ----------
        connection_mode : {pb.GUI, pb.DIRECT}
            Indicates whether pybullet simulator should generate GUI or not

        Returns
        -------
        client : int
            Identifier used to specify simulator client. This is needed when
            making calls because there might be multiple clients running
        '''
        client = pb.connect(connection_mode)
        pb.setPhysicsEngineParameter(numSubSteps=0,
                                     numSolverIterations=100,
                                     solverResidualThreshold=1e-7,
                                     constraintSolverType=pb.CONSTRAINT_SOLVER_LCP_SI,
                                     physicsClientId=client)

        # this path is where we find platform
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane_id = pb.loadURDF('plane.urdf', [0,-0.5,0],
                                    physicsClientId=client)
        pb.changeDynamics(self.plane_id, -1,
                          linearDamping=0.04,
                          angularDamping=0.04,
                          restitution=0,
                          contactStiffness=3000,
                          contactDamping=100,
                          physicsClientId=client)

        return client

    def initialize_robot(self, pos, rot=[0,0,0,1]):
        '''Adds robot to simulator, setting up gripper constraints and initial
        motor commands

        Parameters
        ----------
        pos: array_like
            xyz position, length 3
        rot: array_like
            quaternion, length 4

        Returns
        -------
        int
            id for robot body
        '''
        robot_urdf_path = os.path.join(constants.URDF_DIR, 'xarm.urdf')
        robot_id = pb.loadURDF(robot_urdf_path,
                               pos,
                               rot,
                               flags=pb.URDF_USE_SELF_COLLISION,
                               physicsClientId=self._client)
        self.base_pos = pos
        self.base_rot = rot

        # # set up constraints for linkage in gripper fingers
        for i in [0,1]:
            constraint = pb.createConstraint(robot_id,
                                             self.gripper_joint_ids[i],
                                             robot_id,
                                             self.finger_joint_ids[i],
                                             pb.JOINT_POINT2POINT,
                                             (0,0,0),
                                             (0,0,0.03),
                                             (0,0,0),
                                             physicsClientId= self._client
                                             )
            pb.changeConstraint(constraint, maxForce=1000000)

        # reset joints in hand so that constraints are satisfied
        hand_joint_ids = self.gripper_joint_ids + self.dummy_joint_ids + self.finger_joint_ids
        hand_rest_states = [0.05, 0.05, 0.055, 0.0155, 0.031]
        [pb.resetJointState(robot_id, j_id, jpos, physicsClientId=self._client)
                 for j_id,jpos in zip(hand_joint_ids, hand_rest_states)]

        # allow finger and linkages to move freely
        pb.setJointMotorControlArray(robot_id,
                                     self.dummy_joint_ids+self.finger_joint_ids,
                                     pb.POSITION_CONTROL,
                                     forces=[0,0,0],
                                     physicsClientId= self._client)

        # # make arm joints rigid
        pb.setJointMotorControlArray(robot_id,
                                     self.arm_joint_ids,
                                     pb.POSITION_CONTROL,
                                     5*[0],
                                     positionGains=5*[0.2],
                                     physicsClientId= self._client)

        return robot_id

    def get_hand_pose(self):
        '''Get position and orientation of hand (i.e. where grippers would close)

        This is not the same as the hand link, instead we are interested in the
        space where the grippers would engage with an object

        Returns
        -------
        ndarray
            position vector; shape=(3,); dtype=float
        ndarray
            quaternion; shape=(3,); dtype=float
        '''
        link_state = pb.getLinkState(self.robot_id,
                                     self.end_effector_link_index,
                                     physicsClientId=self._client)
        pos = link_state[4]
        rot = link_state[5]
        return pos, rot

    def _get_link_pose(self, link_name):
        '''Returns position and orientation of robot's link

        Parameters
        ----------
        link_name: str
            name of link in urdf. it should not include "_link" at end of name

        Returns
        -------
        pos: ndarray
            3D position; shape=(3,); dtype=float
        rot: ndarray
            euler angle; shape=(3,); dtype=float
        '''
        assert link_name in self.link_names
        link_index = self.link_names.index(link_name)
        link_state = pb.getLinkState(self.robot_id, link_index,
                                     physicsClientId=self._client)
        pos = link_state[4]
        rot = pb.getEulerFromQuaternion(link_state[5])
        return pos, rot

    def reset_base_pose(self, pos, rot=(0,0,0,1)):
        '''Resets position and orientation of robot base.

        Parameters
        ----------
        pos: array_like
            xyz position, length 3
        rot: array_like
            quaternion, length 4
        '''
        pb.resetBasePositionAndOrientation(self.robot_id, pos, rot,
                                           physicsClientId=self._client)
        self.base_pos = pos
        self.base_rot = rot

    def close(self):
        pb.disconnect(self._client)
