import pybullet as pb
import numpy as np

class Collision:
    def __init__(self, contact_pt, pb_client):
        '''Performs collision detection and inverse kinematics in
        pybullet simulator.

        Parameters
        ----------
        contact_pt : list
            Data about collision involving robot body.  See
            pybullet.getContactPoints for details

        pb_client : int
            Physics client id for the pybullet simulator

        Note
        ----
        This class assumes that the contact points were generated with
        bodyA as the robot body
        '''
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
        if self.self_collision:
            other_link_name = pb.getJointInfo(self.other_body, self.other_link,
                                              physicsClientId=self.pb_client)[12].decode('ascii')
            return f"Collision between xarm:{robot_link_name} and xarm:{other_link_name}"
        else:
            other_body_name = pb.getBodyInfo(self.other_body,
                                              physicsClientId=self.pb_client)[1].decode('ascii')
            return f"Collision between xarm:{robot_link_name} and {other_body_name}"

class MotionPlanner:
    def __init__(self, pb_sim, workspace=None):
        '''Performs collision detection and inverse kinematics in
        pybullet simulator.

        Note
        ----
        This class does not create a pybullet simulater instance, but works on
        an existing one.  This way, the virtual robot can be controlled without
        needing an additional simulator running in the background.

        Parameters
        ----------
        pb_sim : PybulletSimulator
            Simulator object in which the motion planning features
            will be applied
        workspace : array_like, optional
            Array describing the workspace limits for each dimension, must be
            of the form ((min_x, max_x),(min_y, max_y),(min_z, max_z)). If not
            provided, default value is used.  The workspace is used to limit
            where the position of the hand.
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

        Parameters
        ----------
        pos : array_like
            Cartesian (xyz) position of the hand.  Here, the hand means the
            virtual palm link, e.g. the location where the grippers touch when
            the gripper is closed.

        Returns
        -------
        bool
            True if hand is safe, False otherwise.  Safe means it is located
            within the workspace (see mp.__init__ funtion).
        '''
        return np.bitwise_and(pos > self.workspace[:,0],
                              pos < self.workspace[:,1]).all()

    def is_safe_arm_jpos(self, jpos):
        '''Checks if joint positions are within limits

        Parameters
        ----------
        jpos : array_like
            Joint angles for arm servos in order from base to wristRotation

        Returns
        -------
        bool
            True if arm joint configuration is safe, False otherwise.  Safe
            means all joint angles are within specified limits.
        '''
        return np.bitwise_and(jpos > self.arm_joint_limits[:,0],
                              jpos < self.arm_joint_limits[:,1]).all()

    def is_collision_free(self, jpos, ignore_gripper=True):
        '''Checks if robot configuration is free of collisions with other bodies
        in the simulator.

        Parameters
        ----------
        jpos : array_like
            Joint angles of arm servos. Collision checking takes place after
            teleporting robot to these joint angles (prior robot configuration
            is restored at the end)
        ignore_gripper : bool
            True if gripper links are not included in the collision checking.
            Gripper links include left and right finger, hand assebly and
            gripper servo.

        Returns
        -------
        bool
            True if there are no collisions present, False otherwise
        list(obj)
            List of Collision objects describing what collisions are present
        '''
        current_joint_states = self.get_joint_states()

        self._teleport_arm(jpos)
        collisions = self.find_collisions(jpos, ignore_gripper)

        # reset arm to previous joint states
        self.set_joint_states(current_joint_states)
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
                     init_arm_jpos=(0,-0.1,0.1,0,0)
                     ):
        current_joint_states = self.get_joint_states()

        n_arm_joints = len(self.arm_joint_ids)

        self._teleport_arm(init_arm_jpos)
        for _ in range(n_iters_outer):
            jpos = pb.calculateInverseKinematics(self.robot_id,
                                                 self.end_effector_link_index,
                                                 pos,
                                                 rot,
                                                 maxNumIterations=n_iters_inner,
                                                 jointDamping=self.n_joints*[jd],
                                                 physicsClientId=self._client
                                                )
            self._teleport_arm(jpos[:n_arm_joints])

        solved_pos, solved_rot = self.pb_sim.get_hand_pose()

        info = {
            'ik_pos' : solved_pos,
            'ik_rot' : solved_rot,
            'ik_pos_error' : np.linalg.norm(np.subtract(pos, solved_pos)),
            'ik_rot_error' : 0,
        }
        if rot is not None:
            qd = pb.getDifferenceQuaternion(rot, solved_rot)
            info['ik_rot_error'] = 2 * np.arctan2(np.linalg.norm(qd[:3]), qd[3])

        # reset arm to previous joint states
        self.set_joint_states(current_joint_states)
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

    def _teleport_arm(self, arm_jpos):
        '''Resets joint states of arm joints

        Parameters
        ----------
        arm_jpos : array_like, optional
            joint positions for all arm joints in radians; shape=(5,); dtype=float
        '''
        [pb.resetJointState(self.robot_id, i, jp, physicsClientId=self._client)
            for i,jp in zip(self.arm_joint_ids, arm_jpos)]

    def _teleport_gripper(self, state):
        '''Resets joint states of gripper joints

        Parameters
        ----------
        state : float
            gripper state; in range [0,1]
        '''
        jpos = (1-state)*self.gripper_joint_limits[0] + state*self.gripper_joint_limits[1]
        [pb.resetJointState(self.robot_id, i, jp, physicsClientId=self._client)
             for i,jp in zip(self.gripper_joint_ids, jpos)]

        # reset joints of fingers such that the constraints are perfectly satisfied
        z_val = np.cos(jpos[0])*np.subtract(*self.dummy_joint_limits[::-1])[0] \
                    + self.dummy_joint_limits[0,0]
        pb.resetJointState(self.robot_id, self.dummy_joint_ids[0], z_val,
                           physicsClientId=self._client)
        y_val = np.sin(jpos[0])*np.subtract(*self.finger_joint_limits[::-1,0]) \
                    + self.finger_joint_limits[0,0]
        pb.resetJointState(self.robot_id, self.finger_joint_ids[0], y_val,
                           physicsClientId=self._client)
        sep_val = 2 * (y_val - self.finger_joint_limits[0,0]) \
                     + self.finger_joint_limits[0,1]
        pb.resetJointState(self.robot_id, self.finger_joint_ids[1], sep_val,
                           physicsClientId=self._client)

    def find_collisions(self, arm_jpos, ignore_gripper=False):
        '''Returns True if collisions present as arm joint position

        Parameters
        ----------
        arm_jpos : array_like, optional
            joint positions for all arm joints in radians; shape=(5,); dtype=float
        ignore_gripper : bool, default to False
            if True, collisions that involve gripper links will not be returned

        Returns
        -------
        list of Collision objects
        '''
        self._teleport_arm(arm_jpos)

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

    def get_joint_states(self):
        '''Get joint states for arm and gripper'''
        return pb.getJointStates(self.robot_id,
                                 self.all_joint_ids,
                                 self._client)

    def set_joint_states(self, joint_states):
        '''Sets joint states of arm and gripper'''
        for j_state, j_id in zip(joint_states, self.all_joint_ids):
            pb.resetJointState(self.robot_id,
                               j_id,
                               j_state[0],
                               j_state[1],
                               self._client)

    def get_client(self):
        '''Returns pybullet client id used by simulator'''
        return self._client

    def _unpack_simulator_params(self):
        self.robot_id = self.pb_sim.robot_id
        self._client = self.pb_sim._client
        self.n_joints = self.pb_sim.n_joints
        self.arm_joint_ids = self.pb_sim.arm_joint_ids
        self.arm_joint_limits = self.pb_sim.arm_joint_limits

        self.gripper_joint_ids = self.pb_sim.gripper_joint_ids
        self.dummy_joint_ids = self.pb_sim.dummy_joint_ids
        self.finger_joint_ids = self.pb_sim.finger_joint_ids

        self.gripper_joint_limits = self.pb_sim.gripper_joint_limits
        self.dummy_joint_limits = self.pb_sim.dummy_joint_limits
        self.finger_joint_limits = self.pb_sim.finger_joint_limits

        self.link_names = self.pb_sim.link_names
        self.joint_ll = self.pb_sim.joint_ll
        self.joint_ul = self.pb_sim.joint_ul
        self.end_effector_link_index = self.pb_sim.end_effector_link_index

        self.gripper_link_ids = list(self.pb_sim.gripper_joint_ids) \
                                 + list(self.pb_sim.dummy_joint_ids) \
                                 + list(self.pb_sim.finger_joint_ids)

        self.all_joint_ids = list(self.arm_joint_ids) + list(self.gripper_link_ids)

