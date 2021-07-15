import pybullet as pb
import numpy as np

class Collision:
    def __init__(self, contact_pt, pb_client):
        # assumes bodyA is the robot
        self.pb_client = pb_client
        self.robot_body = contact_pt[1]
        self.other_body = contact_pt[2]
        self.robot_link = contact_pt[3]
        self.other_link = contact_pt[4]
        self.self_collision = self.other_body == self.robot_body

    def check(self, ignored_link_ids):
        '''Return True if collision is valid, False if it is to be ignored
        '''
        if self.robot_link not in ignored_link_ids:
            return True
        if self.self_collision and self.other_link not in ignored_link_ids:
            return True
        return False

    def __str__(self):
        robot_link_name = pb.getJointInfo(self.robot_body, self.robot_link,
                                          physicsClientId=self.pb_client)[12].decode('ascii')
        if self.collision:
            other_link_name = pb.getJointInfo(self.other_body, self.other_link,
                                              physicsClientId=self.pb_client)[12].decode('ascii')
            return f"Collision between xarm:{robot_link_name} and xarm:{other_link_name}"
        else:
            other_body_name = pb.getBodyInfo(self.other_body,
                                              physicsClientId=self.pb_client)[1].decode('ascii')
            return f"Collision between xarm:{robot_link}_link and {other_body_name}"

class MotionPlanner:
    def __init__(self, pb_sim, workspace=None):
        '''Class that performs collision detection, inverse kinematics using
        pybullet simulator
        '''
        # unpack info needed to probe the pybullet simulator
        self.pb_sim = pb_sim
        self._unpack_simulator_params()

        if workspace is None:
            self.workspace = np.array(((0.08,0.30),
                                       (-0.18,0.18),
                                       (-0.1, 0.35)))
        else:
            self.workspace = np.array(workspace)
            assert workspace.shape == (3,2), \
                    "Invalid workspace: must be of shape 3x2"
            assert (workspace[:,0] <= workspace[:,1]).all(), \
                    "Invalid workspace: first column must be less than second column"

    def is_safe_hand_position(self, pos):
        '''Checks if hand position is within workspace
        '''
        return np.bitwise_and(pos > self.workspace[:,0],
                              pos < self.workspace[:,1]).all()

    def is_safe_arm_jpos(self, jpos):
        '''Checks if joint positions are within limits
        '''
        return np.bitwise_and(jpos > self.arm_joint_limits[:,0],
                              jpos < self.arm_joint_limits[:,1]).all()

    def is_collision_free(self, jpos, ignore_gripper=True):
        '''Checks if configuration is collision free
        '''
        self._teleport_arm(jpos)
        collisions = self.find_collisions(jpos, ignore_gripper)
        return len(collisions) == 0, collisions

    def is_collision_free_trajectory(self,
                                     start_jpos,
                                     end_jpos,
                                     ignore_gripper=True,
                                     n_substeps=10):
        '''Checks if a trajectory is free from collisions, by checking a set of
        intermediate configurations

        Parameters
        ----------
        start_jpos : array_like of float
            joint positions of arm at start of trajectory
        end_jpos : array_like of float
            joint positions of arm at end of trajectory
        ignore_gripper : bool
        n_substeps : int, default=10
            number of collision checking samples taken within trajectory

        '''
        #TODO: handle trajectories with differing servo speeds
        substeps = np.linspace(start_jpos, end_jpos, num=n_substeps, endpoint=True)

        for jpos in substeps:
            is_free, collision_info = self.is_collision_free(jpos, ignore_gripper)
            if not is_free:
                return is_free, collision_info

        return True, []

    def calculate_ik(self,
                     pos,
                     rot=None,
                     n_iters_outer=3,
                     n_iters_inner=50,
                     jd=0.005,
                     residual_threshold=1e-4,
                     ):
        #https://github.com/bulletphysics/bullet3/issues/1380
        n_arm_joints = len(self.arm_joint_ids)

        for _ in range(n_iters_outer):
            jpos = pb.calculateInverseKinematics(self.robot_id,
                                                 self.end_effector_link_index,
                                                 pos,
                                                 rot,
                                                 maxNumIterations=n_iters_inner,
                                                 # residualThreshold=residual_threshold,
                                                 jointDamping=self.n_joints*[jd],
                                                 physicsClientId=self._client
                                                )
            self._teleport_arm(jpos[:n_arm_joints])

        solved_pos, solved_rot = self.pb_sim.get_hand_pose()
        info = {
            'ik_pos' : solved_pos,
            'ik_rot' : solved_rot,
            'ik_pos_error' : np.linalg.norm(np.subtract(pos, solved_pos)),
            #TODO: add rotation error
        }

        return jpos[:n_arm_joints], info

    def mirror(self, arm_jpos=None, gripper_state=None):
        '''Set simulators joint state to some desired joint state

        Parameters
        ----------
        arm_jpos : array_like, optional
            joint positions for all arm joints in radians; shape=(5,); dtype=float
        gripper_state : float, optional
            gripper state, should be in range from 0 to 1
        '''
        if arm_jpos is not None:
            self._teleport_arm(arm_jpos)
        if gripper_state is not None:
            self._teleport_gripper(gripper_state)

    def _teleport_arm(self, jpos):
        '''Resets joint states of arm joints
        '''
        [pb.resetJointState(self.robot_id, i, jp, physicsClientId=self._client)
            for i,jp in zip(self.arm_joint_ids, jpos)]
        # pb.setJointMotorControlArray(self.robot_id,
                                     # self.arm_joint_ids,
                                     # pb.POSITION_CONTROL,
                                     # jpos,
                                     # positionGains=len(self.arm_joint_ids)*[0.05],
                                     # physicsClientId=self._client)

    def _teleport_gripper(self, state):
        '''Resets joint states of gripper joints
        '''
        jpos = state*self.gripper_joint_limits[0] + (1 - state)*self.gripper_joint_limits[1]

        [pb.resetJointState(self.robot_id, i, jp, physicsClientId=self._client)
             for i,jp in zip(self.gripper_joint_ids, jpos)]
        # pb.setJointMotorControlArray(self.robot_id,
                                     # self.gripper_joint_ids,
                                     # pb.POSITION_CONTROL,
                                     # 2*[jpos],
                                     # positionGains=2*[0.05],
                                     # physicsClientId=self._client,
                                    # )

    def find_collisions(self, jpos, ignore_gripper=False):
        '''Returns True if collisions present as arm joint position

        Parameters
        ----------
        ignore_gripper : bool, default to False

        Returns
        -------
        list of Collision objects
        '''
        self._teleport_arm(jpos)

        ignored_link_ids = set(self.gripper_link_ids) if ignore_gripper else set()

        pb.performCollisionDetection(self._client)
        contact_points = pb.getContactPoints(bodyA=self.robot_id,
                                             physicsClientId=self._client)

        collisions = []
        for cont_pt in contact_points:
            new_collision = Collision(cont_pt, self._client)
            if new_collision.check(ignored_link_ids):
                collisions.append(new_collision)

        return collisions

    def _unpack_simulator_params(self):
        self.robot_id = self.pb_sim.robot_id
        self._client = self.pb_sim._client
        self.n_joints = self.pb_sim.n_joints
        self.arm_joint_ids = self.pb_sim.arm_joint_ids
        self.arm_joint_limits = self.pb_sim.arm_joint_limits
        self.gripper_joint_ids = self.pb_sim.gripper_joint_ids
        self.gripper_joint_limits = self.pb_sim.gripper_joint_limits
        self.link_names = self.pb_sim.link_names
        self.joint_ll = self.pb_sim.joint_ll
        self.joint_ul = self.pb_sim.joint_ul
        self.end_effector_link_index = self.pb_sim.end_effector_link_index

        self.gripper_link_ids = list(self.pb_sim.gripper_joint_ids) \
                                 + list(self.pb_sim.linkage_joint_ids) \
                                 + list(self.pb_sim.finger_joint_ids)

