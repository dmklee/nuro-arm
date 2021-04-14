import numpy as np
import pybullet as pb
import pybullet_data

class MotionPlanner:
    '''Handles IK, FK, collision detection
    '''
    ROBOT_URDF_PATH = "src/assets/xarm.urdf"
    # CAMERA_URDF_PATH = "src/assets/xarm.urdf"
    def __init__(self, connection_mode):
        self._init_pybullet(connection_mode)
        self.link_names = self._get_link_names()
        self.joint_names = self._get_joint_names()
        self.end_effector_index = self.link_names.index('hand')
        self.arm_joint_idxs = [1,2,3,4,5]
        self.arm_jpos_home = np.zeros(len(self.arm_joint_idxs))
        self.gripper_joint_idxs = [6,7]

    def calculate_ik(self, pos, rot=None):
        if rot is not None and len(rot) == 3:
            rot = pb.getQuaternionFromEuler(rot)
        return pb.calculateInverseKinematics(self.id,
                                             self.end_effector_link_index,
                                             pos, rot)[:self.end_effector_link_index]

    def get_link_pose(self, link_name):
        assert link_name in self.link_names
        link_index = self.link_names.index(link_name)
        link_state = pb.getLinkState(self.robot_id, link_index)
        pos = link_state[4]
        rot = pb.getEulerFromQuaternion(link_state[5])
        return pos, rot

    def _init_pybullet(self, connection_mode):
        assert connection_mode in (pb.DIRECT, pb.GUI)
        self._client = pb.connect(connection_mode)

        # this path is where we find platform
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.loadURDF('plane.urdf', [0,0.5,0])

        # add xarm urdf
        self.robot_id = pb.loadURDF(self.ROBOT_URDF_PATH, [0,0,0],[0,0,0,1],
                                  flags=pb.URDF_USE_SELF_COLLISION)

    def _get_joint_names(self):
        num_joints = pb.getNumJoints(self.robot_id)

        joint_names = [pb.getJointInfo(self.robot_id, j_idx)[1].decode("utf-8")
                           for j_idx in range(num_joints)]

        # remove "_joint" from name
        joint_names = [name.replace('_joint','') for name in joint_names]

        return joint_names

    def _get_link_names(self):
        num_joints = pb.getNumJoints(self.robot_id)

        link_names = [pb.getJointInfo(self.robot_id, j_idx)[12].decode("utf-8")
                           for j_idx in range(num_joints)]

        link_names = [name.replace('_link','') for name in link_names]

        return link_names

    def _teleport_arm(self, jpos):
        [pb.resetJointState(self.id, i, jp)
            for i,jp in zip(self.arm_joint_idxs, jpos)]

    def _monitor_movement(self, j_idxs, target, read_fn, max_iter=100, atol=1e-4):
        it = 0

        old_jpos = read_fn()
        while not np.allclose(old_jpos, target, atol=atol):
            pb.stepSimulation()
            it += 1

            jpos = read_fn()
            if it > max_iter or np.allclose(jpos, old_jpos):
                # motion has stopped, so failure
                return False
            old_jpos = jpos

        return True

    def _add_camera_collision_obj(self, pos, rot):
        pass

    def _change_connection_mode(self, connection_mode):
        self._init_pybullet(connection_mode)
