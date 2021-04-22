import pybullet_data
import pybullet as pb
import numpy as np

from neu_ro_arm.camera.camera_utils import rotmat2euler

class UnsafeTrajectoryError(Exception):
    '''A collision was detected at some intermediate joint position in trajectory
    '''
    pass

class UnsafeJointPosition(Exception):
    '''A collision was detected at specified joint position
    '''
    pass

class PybulletBase:
    ROBOT_URDF_PATH = "neu_ro_arm/assets/urdf/xarm.urdf"
    CAMERA_URDF_PATH = "neu_ro_arm/assets/urdf/camera.urdf"
    ROD_URDF_PATH = "neu_ro_arm/assets/urdf/camera_rod.urdf"
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
        self.end_effector_link_index = self.link_names.index('hand')
        self.arm_joint_idxs = [1,2,3,4,5]
        self.gripper_joint_idxs = [6,7]

        self.gripper_closed = np.array([-0.3, -0.3])
        self.gripper_opened = np.array([0.3, 0.3])
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

    def add_camera(self, cam_pose_mtx):
        '''Adds or moves collision object to simulator where camera is located.

        Currently, this assumes the camera is located to the left of the robot
        (from the pov of the robot). Future version could correct for this by
        checking the translation vector component of pose matrix

        Parameters
        ----------
        cam_pose_mtx: ndarray
            Transformation matrix from world frame to camera frame; shape=(4,4);
            dtype=float
        '''
        if self.camera_exists:
            print('Camera already detected. Existing camera will be re-positioned.')
        else:
            self.camera_id = pb.loadURDF(self.CAMERA_URDF_PATH, [0,0,0],[0,0,0,1],
                                        physicsClientId=self._client)
            self.rod_id = pb.loadURDF(self.ROD_URDF_PATH, [0,0,0],[0,0,0,1],
                                     physicsClientId=self._client)

        self._set_camera_pose(cam_pose_mtx)

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

    def _set_camera_pose(self, cam_pose_mtx):
        '''Changes location of camera-related collision object

        Parameters
        ----------
        cam_pose_mtx: ndarray
            Transformation matrix from world frame to camera frame; shape=(4,4);
            dtype=float
        '''
        pos = cam_pose_mtx[:3,3]/1000.
        rotmat = cam_pose_mtx[:3,:3]
        quat = pb.getQuaternionFromEuler(rotmat2euler(rotmat))
        pb.resetBasePositionAndOrientation(self.camera_id, pos, quat,
                                          physicsClientId=self._client)

        rod_offset_vec = np.array((0.026, -0.012, -0.013))
        rod_pos = pos + np.dot(rotmat, rod_offset_vec)
        rod_pos[2] = 0
        pb.resetBasePositionAndOrientation(self.rod_id, rod_pos, (0,0,0,1),
                                          physicsClientId=self._client)

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
            True if returned joint positions will result in collision
        ndarray
            joint position of arm joints; shape=(5,); dtype=float
        dict
            additional information about positional and rotational error of
            IK solution
        '''
        #TODO: return data on achieved pos, rot, and error
        #       set max error thresholds maybe?
        if rot is not None:
            rot = pb.getQuaternionFromEuler(rot)
        jpos = pb.calculateInverseKinematics(self.robot_id,
                                             self.end_effector_link_index,
                                             pos,
                                             rot,
                                             physicsClientId=self._client,
                                            )

        arm_jpos = jpos[:self.end_effector_link_index]

        data = {}

        is_collision = self._check_collisions(arm_jpos)
        return is_collision, arm_jpos, data

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
            jpos = gripper_state*self.gripper_opened \
                    + (1-gripper_state)*self.gripper_closed

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
        '''
        assert len(target_jpos) == len(self.arm_joint_idxs)
        current_jpos = [pb.getJointState(self.robot_id, j_idx, physicsClientId=self._client)[0]
                            for j_idx in self.arm_joint_idxs]

        inter_jpos = np.linspace(current_jpos, target_jpos,
                                 num=num_steps, endpoint=True)

        safe = True
        for jpos in inter_jpos[1:]:
            if self._check_collisions(jpos):
                safe = False
                break

        self._teleport_arm(current_jpos)
        return True

    def _check_collisions(self, jpos):
        '''Returns True if collisions present as arm joint position
        '''
        self._teleport_arm(jpos)
        pb.performCollisionDetection(self._client)
        for cont_pt in pb.getContactPoints(physicsClientId=self._client):
            bodies = (cont_pt[1], cont_pt[2])
            links = (cont_pt[3], cont_pt[4])
            if self.robot_id in bodies:
                body_idx = bodies.index(self.robot_id)
                robot_link = links[body_idx]
                if self.link_names[robot_link] != 'base':
                    return True

        return False

if __name__ == "__main__":
    import constants as constants
    mp = MotionPlanner(constants.default_cam_pose_mtx)
    import time
    state = True
    while True:
        mp._teleport_gripper(state)
        state = not state
        time.sleep(1)
