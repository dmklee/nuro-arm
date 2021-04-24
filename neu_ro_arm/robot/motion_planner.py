import pybullet_data
import pybullet as pb
import numpy as np

from neu_ro_arm.camera.camera_utils import rotmat2euler
from neu_ro_arm.constants import GRIPPER_CLOSED, GRIPPER_OPENED

class UnsafeTrajectoryError(Exception):
    '''A collision was detected at some intermediate joint position in trajectory
    '''
    def __init__(self, robot_link, other_body, **kwargs):
        msg = f"Collision detected between robot:{robot_link}_link and {other_body}"
        super().__init__(msg)

class UnsafeJointPosition(Exception):
    '''A collision was detected at specified joint position
    '''
    def __init__(self, robot_link, other_body, **kwargs):
        msg = f"Collision detected between robot:{robot_link}_link and {other_body}"
        super().__init__(msg)

class PybulletBase:
    ROBOT_URDF_PATH = "neu_ro_arm/assets/urdf/xarm.urdf"
    CAMERA_URDF_PATH = "neu_ro_arm/assets/urdf/camera.urdf"
    ROD_URDF_PATH = "neu_ro_arm/assets/urdf/camera_rod.urdf"
    CUBE_URDF_PATH = "neu_ro_arm/assets/urdf/cube.urdf"
    def __init__(self, connection_mode):
        '''Base class for pybullet simulator to handle initialization and attributes
        about robot joints

        Parameters
        ----------
        connection_mode : {pb.GUI, pb.DIRECT}
            Indicates whether pybullet simulator should generate GUI or not

        Attributes
        ----------
        link_names : list of str
            names of links in robot urdf
        joint_names : list of str
            names of joints in robot urdf
        end_effector_link_index : int
            index of hand link, this is used to specify which link
            should be used for IK
        arm_joint_idxs: list of int
            indices of joints that control the arm
        gripper_joint_idxs: list of int
            indices of two gripper joints. for simulator, gripper operation is
            controlled by two joints which should both move in concert to reflect
            the real xArm
        gripper_closed : ndarray
            joint positions of gripper joints that result in closed gripper in
            the simulator; shape = (2,); dtype=float
        gripper_opened : ndarray
            joint positions of gripper joints that result in opened gripper in
            the simulator; shape = (2,); dtype=float
        camera_exists : bool
            True if camera collision object has been added to simulator
        '''
        self._client = self._init_pybullet(connection_mode)
        self.link_names = self._get_link_names()
        self.joint_names = self._get_joint_names()

        self.arm_joint_idxs = [1,2,3,4,5]
        self.end_effector_link_index = 6
        self.gripper_joint_idxs = [7,8]

        self.gripper_closed = np.array([0.0, 0.0])
        self.gripper_opened = np.array([0.042, 0.042])
        self.camera_exists = False

    def _init_pybullet(self, connection_mode):
        '''Creates pybullet simulator and loads world plane and robot.

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

        # this path is where we find platform
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        # shift plane by 0.5 m to put workspace on single colored tile
        self.plane_id = pb.loadURDF('plane.urdf', [0,0.5,0],
                                    physicsClientId=client)

        self.robot_id = pb.loadURDF(self.ROBOT_URDF_PATH, [0,0,0],[0,0,0,1],
                                    flags=pb.URDF_USE_SELF_COLLISION,
                                    physicsClientId=client)

        return client

    def add_camera(self, pose_mtx):
        '''Adds or moves collision object to simulator where camera is located.

        Currently, this assumes the camera is located to the left of the robot
        (from the pov of the robot). Future version could correct for this by
        checking the translation vector component of pose matrix

        Parameters
        ----------
        pose_mtx: ndarray
            Transformation matrix from world frame to camera frame; shape=(4,4);
            dtype=float
        '''
        cam_pos, cam_quat, rod_pos, rod_quat = self._unpack_camera_pose(pose_mtx)
        if self.camera_exists:
            print('Camera already detected. Existing camera will be re-positioned.')
            pb.resetBasePositionAndOrientation(self.camera_id, cam_pos, cam_quat,
                                              physicsClientId=self._client)
            pb.resetBasePositionAndOrientation(self.rod_id, rod_pos, rod_quat,
                                              physicsClientId=self._client)
        else:
            self.camera_id = pb.loadURDF(self.CAMERA_URDF_PATH, cam_pos, cam_quat,
                                        physicsClientId=self._client)
            self.rod_id = pb.loadURDF(self.ROD_URDF_PATH, rod_pos, rod_quat,
                                     physicsClientId=self._client)


    def _get_joint_names(self):
        '''Returns list of joints for robot urdf
        '''
        num_joints = pb.getNumJoints(self.robot_id)

        joint_names = [pb.getJointInfo(self.robot_id, j_idx, physicsClientId=self._client)[1]
                        for j_idx in range(num_joints)]

        # remove "_joint" from name
        joint_names = [name.decode("utf-8").replace('_joint','') for name in joint_names]

        return joint_names

    def _get_link_names(self):
        '''Returns list of names for each link in robot urdf
        '''
        num_joints = pb.getNumJoints(self.robot_id, physicsClientId=self._client)

        link_names = [pb.getJointInfo(self.robot_id, j_idx, physicsClientId=self._client)[12]
                           for j_idx in range(num_joints)]

        link_names = [name.decode("utf-8").replace('_link','') for name in link_names]

        return link_names

    def _unpack_camera_pose(self, cam_pose_mtx):
        '''Get params for positioning camera and rod based on pose of camera

        Parameters
        ----------
        cam_pose_mtx: ndarray
            Transformation matrix from world frame to camera frame; shape=(4,4);
            dtype=float

        Returns
        -------
        cam_pos : array_like
            3d postion vector of camera body
        cam_quat : array_like
            quaternion of camera body
        rod_pos : array_like
            3d postion vector of rod body
        rod_quat : array_like
            quaternion of rod body
        '''
        cam_pos = cam_pose_mtx[:3,3]
        cam_rotmat = cam_pose_mtx[:3,:3]
        cam_quat = pb.getQuaternionFromEuler(rotmat2euler(cam_rotmat))

        rod_offset_vec = np.array((0.026, -0.012, -0.013))
        rod_pos = cam_pos + np.dot(cam_rotmat, rod_offset_vec)
        rod_pos[2] = 0
        rod_quat = (0,0,0,1)

        return cam_pos, cam_quat, rod_pos, rod_quat

    def add_cube(self, pos, euler):
        '''Add 1" cube to simulator

        Parameters
        ----------
        pos : array_like
            3d position of center of cube
        euler: array_like
            euler angles of cube; shape=(3,)

        Returns
        -------
        int
            body id of cube in simulator
        '''
        quat = pb.getQuaternionFromEuler(euler)
        cube_id = pb.loadURDF(self.CUBE_URDF_PATH,
                              pos,
                              quat,
                              physicsClientId=self._client,
                             )
        return cube_id

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

    def get_hand_pose(self):
        '''Get position and orientation of hand (i.e. where grippers would close)

        This is not the same as the hand link, instead we are interested in the
        space where the grippers would engage with an object

        Returns
        -------
        ndarray
            position vector; shape=(3,); dtype=float
        ndarray
            euler angle; shape=(3,); dtype=float
        '''
        link_state = pb.getLinkState(self.robot_id,
                                     self.end_effector_link_index,
                                     physicsClientId=self._client)
        pos = link_state[4]
        rot = pb.getEulerFromQuaternion(link_state[5])
        return pos, rot

class MotionPlanner(PybulletBase):
    def __init__(self):
        '''Pybullet simulator of robot used to perform inverse kinematics
        and collision detection quickly in background
        '''
        super(MotionPlanner, self).__init__(pb.DIRECT)

    def _calculate_ik(self, pos, rot=None):
        '''Performs inverse kinematics to generate hand link pose

        Pybullet IK is influenced by current joint state so it is best to make
        a mirror call before using this method

        Parameters
        ----------
        pos : array_like
            desired 3D position of end effector
        rot : array_like, optional
            desired euler angle of end effector

        Returns
        -------
        bool
            True if returned joint positions are safe (e.g. do not result in collision)
        ndarray
            joint position of arm joints; shape=(5,); dtype=float
        dict
            collision data, may also include data on accuracy of ik solution
        '''
        if rot is not None:
            rot = pb.getQuaternionFromEuler(rot)

        jpos = pb.calculateInverseKinematics(self.robot_id,
                                             self.end_effector_link_index,
                                             pos,
                                             rot,
                                             maxNumIterations=500,
                                             physicsClientId=self._client,
                                            )

        num_arm_joints = len(self.arm_joint_idxs)
        arm_jpos = jpos[:num_arm_joints]

        is_collision, data = self._check_collisions(arm_jpos)

        # check collision teleports arm so we can check IK solution error here
        achieved_pos, achieved_rot = self.get_hand_pose()
        data['achieved_pos'] = achieved_pos
        data['achieved_rot'] = achieved_rot
        data['pos_error'] = np.linalg.norm(np.subtract(pos, achieved_pos))Difference

        is_safe = not is_collision
        return is_safe, arm_jpos, data

    def mirror(self, arm_jpos=None, gripper_state=None, gripper_jpos=None):
        '''Set simulators joint state to some desired joint state

        Parameters
        ----------
        arm_jpos : array_like, optional
            joint positions for all arm joints in radians; shape=(5,); dtype=float
        gripper_state : float, optional
            gripper state, should be in range from 0 to 1
        gripper_jpos : array_like, optional
            gripper joint position in radians
        '''
        self._teleport_arm(arm_jpos)
        self._teleport_gripper(state=gripper_state,
                               jpos=gripper_jpos
                              )

    def _teleport_arm(self, jpos=None):
        '''Resets joint states of arm joints
        '''
        if jpos is not None:
            [pb.resetJointState(self.robot_id, i, jp, physicsClientId=self._client)
                for i,jp in zip(self.arm_joint_idxs, jpos)]

    def _teleport_gripper(self, state=None, jpos=None):
        '''Resets joint states of gripper joints
        '''
        if state is not None:
            jpos = state*self.gripper_opened \
                    + (1 - state)*self.gripper_closed

        if jpos is not None:
            [pb.resetJointState(self.robot_id, i, jp, physicsClientId=self._client)
                 for i,jp in zip(self.gripper_joint_idxs, jpos)]

    def check_arm_trajectory(self, target_jpos, num_steps=10):
        '''Checks collision of arm links along linear path in joint space

        Parameters
        ----------
        target_jpos : array_like
            joint positions of arm at end of trajectory; shape=(5,); dtype=float
        num_steps : int
            number of collision checking samples taken within trajectory

        Returns
        -------
        bool
            True if trajectory is safe (e.g. no collisions were detected)
        dict
            info on collision if collision occurred
        '''
        assert len(target_jpos) == len(self.arm_joint_idxs)
        current_jpos = [pb.getJointState(self.robot_id, j_idx, physicsClientId=self._client)[0]
                            for j_idx in self.arm_joint_idxs]

        inter_jpos = np.linspace(current_jpos, target_jpos,
                                 num=num_steps, endpoint=True)

        is_safe = True
        for jpos in inter_jpos[1:]:
            collision, data = self._check_collisions(jpos)
            if collision:
                is_safe = False
                break

        self._teleport_arm(current_jpos)
        return is_safe, data

    def _check_collisions(self, jpos, gripper_mode='current'):
        '''Returns True if collisions present as arm joint position

        Parameters
        ----------
        jpos : array_like
            positions of arm joints in radians
        gripper_mode : {'closed', 'open', 'ignore', 'current'}
            how to treat gripper during collision check. if 'current' then the
            gripper position will not be changed

        Returns
        -------
        bool
            True if collision was detected
        dict
            data about collision
        '''
        self._teleport_arm(jpos)

        if gripper_mode == 'open':
            self._teleport_gripper(state=GRIPPER_OPENED)
        elif gripper_mode == 'closed':
            self._teleport_gripper(state=GRIPPER_CLOSED)

        # ignore base because it doesnt move and will likely be in collision
        # with camera at all times
        ignored_links = ['base']
        if gripper_mode == 'ignore':
            ignored_links.extend(['gripper_left', 'gripper_right'])

        pb.performCollisionDetection(self._client)
        contact_points = pb.getContactPoints(bodyA=self.robot_id,
                                             physicsClientId=self._client)
        for cont_pt in contact_points:
            assert cont_pt[1] == self.robot_id
            robot_link = cont_pt[3]
            robot_link_name = self.link_names[robot_link]
            if robot_link_name not in ignored_links:
                other_body_id = cont_pt[2]
                other_body_link = cont_pt[4]
                other_body_name = pb.getBodyInfo(other_body_id,
                                                 physicsClientId=self._client
                                                )[1].decode('ascii')

                return True, {'robot_link' : robot_link_name,
                              'other_body' : other_body_name,
                             }

        return False, {}
