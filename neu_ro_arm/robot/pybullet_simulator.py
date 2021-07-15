import pybullet_data
import pybullet as pb
import numpy as np
from neu_ro_arm import transformation_utils

class PybulletSimulator:
    ROBOT_URDF_PATH = "neu_ro_arm/assets/urdf/xarm.urdf"
    CAMERA_URDF_PATH = "neu_ro_arm/assets/urdf/camera.urdf"
    ROD_URDF_PATH = "neu_ro_arm/assets/urdf/camera_rod.urdf"
    CUBE_URDF_PATH = "neu_ro_arm/assets/urdf/cube.urdf"
    def __init__(self, headless):
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
        camera_exists : bool
            True if camera collision object has been added to simulator
        '''
        self.arm_joint_ids = [1,2,3,4,5]
        self.linkage_joint_ids = [6,9]
        self.gripper_joint_ids = [7,10]
        self.finger_joint_ids = [8,11]
        self.end_effector_link_index = 12

        self.arm_joint_limits = np.array(((-2, -1.58, -2, -1.8, -2),
                                          ( 2,  1.58,  2,  2.0,  2)))

        self.gripper_joint_limits = np.array([0.05, 1.38])

        connection_mode = pb.DIRECT if headless else pb.GUI
        self._initialize(connection_mode)
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

        self.camera_exists = False

    def _initialize(self, connection_mode):
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
        self._client = pb.connect(connection_mode)

        # this path is where we find platform
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane_id = pb.loadURDF('plane.urdf', [0,0.5,0],
                                    physicsClientId=self._client)

        suction_cup_height = 0.012
        self.robot_id = pb.loadURDF(self.ROBOT_URDF_PATH,
                                    [0,0,suction_cup_height],
                                    [0,0,0,1],
                                    flags=pb.URDF_USE_SELF_COLLISION,
                                    physicsClientId=self._client)
        for i in [0,1]:
            pb.resetJointState(self.robot_id, self.gripper_joint_ids[i], np.pi/2)
            pb.resetJointState(self.robot_id, self.linkage_joint_ids[i], np.pi/2)

        # set up constraints for linkage in gripper fingers
        left_linkage_constraint = pb.createConstraint(self.robot_id,
                                                      self.linkage_joint_ids[0],
                                                      self.robot_id,
                                                      self.finger_joint_ids[0],
                                                      pb.JOINT_POINT2POINT,
                                                      (0,0,0),
                                                      (0.03,0,0.0),
                                                      (-0.022,0.0,0.0),
                                                      physicsClientId= self._client
                                                     )
        pb.changeConstraint(left_linkage_constraint, maxForce=10000)
        right_linkage_constraint = pb.createConstraint(self.robot_id,
                                                       self.linkage_joint_ids[1],
                                                       self.robot_id,
                                                       self.finger_joint_ids[1],
                                                       pb.JOINT_POINT2POINT,
                                                       (0,0,1),
                                                       (-0.03,0,0.0),
                                                       (0.022,0.0,0.0),
                                                       physicsClientId= self._client
                                                      )
        pb.changeConstraint(right_linkage_constraint, maxForce=10000)

        # allow finger and linkages to move freely
        pb.setJointMotorControlArray(self.robot_id,
                                     self.linkage_joint_ids+self.finger_joint_ids,
                                     pb.POSITION_CONTROL,
                                     forces=[0,0,0,0])

        # make arm joints rigid
        pb.setJointMotorControlArray(self.robot_id,
                                     self.arm_joint_ids,
                                     pb.POSITION_CONTROL,
                                     5*[0],
                                     positionGains=5*[0.1],
                                    )

        pb.stepSimulation(self._client)

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
        cam_quat = pb.getQuaternionFromEuler(transformation_utils.rotmat2euler(cam_rotmat))

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

    def close(self):
        pb.disconnect(self._client)
