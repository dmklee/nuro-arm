import pybullet_data
import pybullet as pb
import numpy as np
from neu_ro_arm import transformation_utils

class BasePybullet:
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
        self.arm_joint_idxs = [1,2,3,4,5]
        self.linkage_joint_idxs = [6,9]
        self.gripper_joint_idxs = [7,10]
        self.finger_joint_idxs = [8,11]
        self.end_effector_link_index = 12

        self._init_pybullet(connection_mode)

        self.gripper_closed = np.array([0.05, 0.05])
        self.gripper_opened = np.array([1.38, 1.38])
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
        self._client = pb.connect(connection_mode)

        # this path is where we find platform
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane_id = pb.loadURDF('plane.urdf', [0,0.5,0],
                                    physicsClientId=self._client)

        self.robot_id = pb.loadURDF(self.ROBOT_URDF_PATH, [0,0,0],[0,0,0,1],
                                    flags=pb.URDF_USE_SELF_COLLISION | pb.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS,
                                    physicsClientId=self._client)
        for i in [0,1]:
            pb.resetJointState(self.robot_id, self.gripper_joint_idxs[i], np.pi/3)
            pb.resetJointState(self.robot_id, self.linkage_joint_idxs[i], np.pi/3)

        # set up constraints for linkage in gripper fingers
        left_linkage_constraint = pb.createConstraint(self.robot_id,
                                                      self.linkage_joint_idxs[0],
                                                      self.robot_id,
                                                      self.finger_joint_idxs[0],
                                                      pb.JOINT_POINT2POINT,
                                                      (0,0,0),
                                                      (0.03,0,0.0),
                                                      (-0.022,0.00,0.0),
                                                      physicsClientId= self._client
                                                     )
        pb.changeConstraint(left_linkage_constraint, maxForce=1000000)
        right_linkage_constraint = pb.createConstraint(self.robot_id,
                                                       self.linkage_joint_idxs[1],
                                                       self.robot_id,
                                                       self.finger_joint_idxs[1],
                                                       pb.JOINT_POINT2POINT,
                                                       (0,0,0),
                                                       (-0.03,0,0.0),
                                                       (0.014,-0.017,0.0),
                                                       physicsClientId= self._client
                                                      )
        pb.changeConstraint(right_linkage_constraint, maxForce=1000000)

        # allow finger and linkages to move freely
        pb.setJointMotorControlArray(self.robot_id,
                                     self.arm_joint_idxs,
                                     pb.POSITION_CONTROL,
                                     targetPositions=[0,0,0,0,0],
                                    )
        pb.setJointMotorControlArray(self.robot_id,
                                     self.linkage_joint_idxs+self.finger_joint_idxs,
                                     pb.POSITION_CONTROL,
                                     forces=[0,0,0,0])

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

if __name__ == "__main__":
    import time
    sim = BasePybullet(pb.GUI)
    # pb.setGravity(0,0,-10)
    pb.setRealTimeSimulation(1)

    # pb.resetJointState(sim.robot_id, sim.link_names.index('left_finger_bar'), np.pi/3)
    # pb.resetJointState(sim.robot_id, sim.link_names.index('right_finger_bar'), np.pi/3)
    # pb.resetJointState(sim.robot_id, sim.joint_names.index('left_knuckle'), np.pi/3)
    # pb.resetJointState(sim.robot_id, sim.joint_names.index('right_knuckle'), np.pi/3)
    # left_linkage_constraint = pb.createConstraint(sim.robot_id,
                                                  # sim.link_names.index('left_finger_bar'),
                                                  # sim.robot_id,
                                                  # sim.link_names.index('left_finger_tip'),
                                                  # pb.JOINT_POINT2POINT,
                                                  # (0,0,0), (0.03,0,0.0), (-0.022,0.00,0.0),
                                                  # physicsClientId= sim._client
                                                 # )
    # pb.changeConstraint(left_linkage_constraint, maxForce=10000)
    # right_linkage_constraint = pb.createConstraint(sim.robot_id,
                                                  # sim.link_names.index('right_finger_bar'),
                                                  # sim.robot_id,
                                                  # sim.link_names.index('right_finger_tip'),
                                                  # pb.JOINT_POINT2POINT,
                                                  # (0,0,0), (-0.03,0,0.0), (0.014,-0.017,0.0),
                                                  # physicsClientId= sim._client
                                                 # )
    # pb.changeConstraint(right_linkage_constraint, maxForce=10000)


    pb.setJointMotorControlArray(sim.robot_id,
                                 sim.arm_joint_idxs,
                                 pb.POSITION_CONTROL,
                                 targetPositions=[0]*5,
                                )
    # pb.setJointMotorControlArray(sim.robot_id,
                                 # [6,8,9,11],
                                 # pb.POSITION_CONTROL,
                                 # forces=[0]*4,
                                # )
    # for i,j in enumerate(sim.link_names):
        # print(i,j)
    # time.sleep(0.5)
    while 1:
        pb.setJointMotorControlArray(sim.robot_id,
                                     sim.gripper_joint_idxs,
                                     controlMode=pb.POSITION_CONTROL,
                                     targetPositions=[0.05, 0.05],
                                     positionGains=2*[0.01],
                                     velocityGains=2*[0.5],
                                    )
        time.sleep(1.5)
        pb.setJointMotorControlArray(sim.robot_id,
                                     sim.gripper_joint_idxs,
                                     controlMode=pb.POSITION_CONTROL,
                                     targetPositions=[1.38,1.38],
                                     positionGains=2*[0.01],
                                     velocityGains=2*[0.5],
                                    )
        time.sleep(1.5)
