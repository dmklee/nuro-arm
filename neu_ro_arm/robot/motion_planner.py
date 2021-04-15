import numpy as np
import pybullet as pb
import pybullet_data

from camera.camera_utils import rotmat2euler

class MotionPlanner:
    '''Handles IK, FK, collision detection
    '''
    ROBOT_URDF_PATH = "assets/urdf/xarm.urdf"
    CAMERA_URDF_PATH = "assets/urdf/camera.urdf"
    ROD_URDF_PATH = "assets/urdf/camera_rod.urdf"
    def __init__(self, cam_pose_mtx=None):
        self._init_pybullet(cam_pose_mtx)
        self.link_names = self._get_link_names()
        self.joint_names = self._get_joint_names()
        self.end_effector_link_index = self.link_names.index('hand')
        self.arm_joint_idxs = [1,2,3,4,5]
        self.arm_jpos_home = np.zeros(len(self.arm_joint_idxs))
        self.gripper_joint_idxs = [6,7]

    def _calculate_ik(self, pos, rot=None):
        if rot is not None and len(rot) == 3:
            rot = pb.getQuaternionFromEuler(rot)
        return pb.calculateInverseKinematics(self.robot_id,
                                             self.end_effector_link_index,
                                             pos, rot)[:self.end_effector_link_index]

    def _get_link_pose(self, link_name):
        assert link_name in self.link_names
        link_index = self.link_names.index(link_name)
        link_state = pb.getLinkState(self.robot_id, link_index)
        pos = link_state[4]
        rot = pb.getEulerFromQuaternion(link_state[5])
        return pos, rot

    def _init_pybullet(self, cam_pose_mtx):
        self._client = pb.connect(pb.GUI)

        # this path is where we find platform
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane_id = pb.loadURDF('plane.urdf', [0,0.5,0])

        # add xarm urdf
        self.robot_id = pb.loadURDF(self.ROBOT_URDF_PATH, [0,0,0],[0,0,0,1],
                                  flags=pb.URDF_USE_SELF_COLLISION)

        # initialize camera far away so collisions dont occur
        if cam_pose_mtx is not None:
            self.add_camera(cam_pose_mtx)

    def add_camera(self, cam_pose_mtx):
        self.camera_id = pb.loadURDF(self.CAMERA_URDF_PATH, [0,0,0],[0,0,0,1])
        self.rod_id = pb.loadURDF(self.ROD_URDF_PATH, [0,0,0],[0,0,0,1])

        self.set_camera_pose(cam_pose_mtx)

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
        [pb.resetJointState(self.robot_id, i, jp)
            for i,jp in zip(self.arm_joint_idxs, jpos)]

    def check_arm_trajectory(self, target_jpos, num_steps=10):
        '''Checks collision of arm links'''
        assert len(target_jpos) == len(self.arm_joint_idxs)
        current_jpos = np.array([pb.getJointState(self.robot_id, j_idx)[0]
                               for j_idx in self.arm_joint_idxs])
        inter_jpos = np.linspace(current_jpos, target_jpos, num=num_steps, endpoint=True)

        for jpos in inter_jpos[1:]:
            if self._check_collisions(jpos):
                return False

        return True

    def _monitor_movement(self, j_idxs, target, read_fn, max_iter=100, atol=1e-4):
        it = 0

        old_jpos = read_fn()
        while not np.allclose(old_jpos, target, atol=atol):
            it += 1

            jpos = read_fn()
            if it > max_iter or np.allclose(jpos, old_jpos):
                # motion has stopped, so failure
                return False
            old_jpos = jpos

        return True

    def _check_collisions(self, jpos):
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

    def set_camera_pose(self, cam2world):
        pos = cam2world[:3,3]/1000.
        rotmat = cam2world[:3,:3]
        quat = pb.getQuaternionFromEuler(rotmat2euler(rotmat))
        pb.resetBasePositionAndOrientation(self.camera_id, pos, quat)

        rod_offset_vec = np.array((0.026, -0.012, -0.013))
        rod_pos = pos + np.dot(rotmat, rod_offset_vec)
        rod_pos[2] = 0
        pb.resetBasePositionAndOrientation(self.rod_id, rod_pos, (0,0,0,1))

if __name__ == "__main__":
    import constants as constants
    mp = MotionPlanner(constants.default_cam_pose_mtx)
    jpos = np.array([0,0,-np.pi/2,0,0])
    mp._teleport_arm(jpos)
    target = np.array([0,np.pi,-np.pi/2,0,0])
    print(mp.check_arm_trajectory(target, num_steps=2))
    sadf
    import time
    while True:
        print(mp._get_collisions_at(jpos))
        pb.stepSimulation()
        time.sleep(0.1)
