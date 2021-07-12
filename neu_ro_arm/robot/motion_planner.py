import pybullet as pb
import numpy as np

from neu_ro_arm.constants import GRIPPER_CLOSED, GRIPPER_OPENED
from neu_ro_arm.robot.base_pybullet import BasePybullet

class CollisionError(Exception):
    def __init__(self, robot_link, other_body, **kwargs):
        self.robot_link = robot_link
        self.other_body = other_body
        msg = f"Collision detected between xarm:{robot_link}_link and {other_body}"
        super().__init__(msg)

class UnsafeTrajectoryError(CollisionError):
    '''A collision was detected at some intermediate joint position in trajectory
    '''

class UnsafeJointPosition(CollisionError):
    '''A collision was detected at specified joint position
    '''

class ProbitedHandPosition(Exception):
    '''A collision was detected at specified joint position
    '''
    def __init__(self):
        msg = f"This hand position is not allowed for safety reasons."
        super().__init__(msg)

class MotionPlanner(BasePybullet):
    def __init__(self, headless=True):
        '''Pybullet simulator of robot used to perform inverse kinematics
        and collision detection quickly in background
        '''
        connection_mode = pb.DIRECT if headless else pb.GUI
        self.headless = headless
        super(MotionPlanner, self).__init__(connection_mode)
        self._hand_position_limits = np.array(((0.08,0.30),
                                               (-0.18,0.18),
                                               (-0.1, 0.35)))

    def safe_hand_position(self, pos):
        return np.bitwise_and(pos > self._hand_position_limits[:,0],
                              pos < self._hand_position_limits[:,1]).all()

    def calculate_ik(self, pos, rot=None, **ik_kwargs):
        '''Performs inverse kinematics to generate hand link pose

        Pybullet IK is influenced by current joint state so it is best to make
        a mirror call before using this method

        Parameters
        ----------
        pos : array_like
            desired 3D position of end effector
        rot : array_like, optional
            desired quaternion of end effector

        Raises
        ------
        ProbitedHandPosition
            hand position is outside of allowable limits
        UnsafeJointPosition
            joint positions needed to achieve hand pose is in
            collision with environment

        Returns
        -------
        ndarray
            joint position of arm joints; shape=(5,); dtype=float
        dict
            data on achieved pose and error
        '''
        if not self.safe_hand_position(pos):
            raise ProbitedHandPosition

        jpos = self._iterative_ik(pos, rot, **ik_kwargs)
        arm_jpos = jpos[:len(self.arm_joint_idxs)]

        is_collision, collision_data = self._check_collisions(arm_jpos)
        if is_collision:
            raise UnsafeJointPosition(**collision_data)

        # check collision teleports arm so we can check IK solution error here
        achieved_pos, achieved_rot = self.get_hand_pose()
        data = dict()
        data['achieved_pos'] = achieved_pos
        data['achieved_rot'] = achieved_rot
        data['pos_error'] = np.linalg.norm(np.subtract(pos, achieved_pos))

        return arm_jpos, data

    def _iterative_ik(self,
                      pos,
                      rot,
                      n_iters_outer=3,
                      n_iters_inner=50,
                      jd=0.01,
                     ):
        # TODO: add threshold to stop iteration
        #https://github.com/bulletphysics/bullet3/issues/1380
        n_arm_motors = len(self.arm_joint_idxs)
        for _ in range(n_iters_outer):
            jpos = pb.calculateInverseKinematics(self.robot_id,
                                                 self.end_effector_link_index,
                                                 pos,
                                                 rot,
                                                 maxNumIterations=n_iters_inner,
                                                 jointDamping=self.num_joints*[jd],
                                                 physicsClientId=self._client
                                                )
            # early stoppage would occur here

            self._teleport_arm(jpos[:n_arm_motors])

        return jpos[:n_arm_motors]

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
            for i,jp in zip(self.arm_joint_idxs, jpos)]
        pb.setJointMotorControlArray(self.robot_id,
                                     self.arm_joint_idxs,
                                     pb.POSITION_CONTROL,
                                     jpos,
                                     positionGains=len(self.arm_joint_idxs)*[0.05],
                                     physicsClientId=self._client)

    def _teleport_gripper(self, state):
        '''Resets joint states of gripper joints
        '''
        jpos = state*self.gripper_opened + (1 - state)*self.gripper_closed

        [pb.resetJointState(self.robot_id, i, jp, physicsClientId=self._client)
             for i,jp in zip(self.gripper_joint_idxs, jpos)]
        pb.setJointMotorControlArray(self.robot_id,
                                     self.gripper_joint_idxs,
                                     pb.POSITION_CONTROL,
                                     jpos,
                                     positionGains=2*[0.05],
                                     physicsClientId=self._client,
                                    )

    def check_arm_trajectory(self, target_jpos, num_steps=10):
        '''Checks collision of arm links along linear path in joint space

        Parameters
        ----------
        target_jpos : array_like
            joint positions of arm at end of trajectory; shape=(5,); dtype=float
        num_steps : int
            number of collision checking samples taken within trajectory

        Raises
        -------
        UnsafeTrajectoryError
            some joint configuration along trajectory results in collision
        '''
        assert len(target_jpos) == len(self.arm_joint_idxs)
        current_jpos = [pb.getJointState(self.robot_id, j_idx, physicsClientId=self._client)[0]
                            for j_idx in self.arm_joint_idxs]

        inter_jpos = np.linspace(current_jpos, target_jpos,
                                 num=num_steps, endpoint=True)

        is_collision = False
        for jpos in inter_jpos[1:]:
            is_collision, collision_data = self._check_collisions(jpos)
            if is_collision:
                break

        self._teleport_arm(current_jpos)
        if is_collision:
            raise UnsafeTrajectoryError(**collision_data)

    def _check_collisions(self, jpos=None, gripper_mode='ignore'):
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
        if jpos is not None:
            self._teleport_arm(jpos)

        if gripper_mode == 'open':
            self._teleport_gripper(state=GRIPPER_OPENED)
        elif gripper_mode == 'closed':
            self._teleport_gripper(state=GRIPPER_CLOSED)

        # ignore base because it doesnt move and will likely be in collision
        # with camera at all times
        if gripper_mode == 'ignore':
            ignored_links = ['left_finger_base',
                             'left_finger_linkage',
                             'left_finger_tip',
                             'right_finger_base',
                             'right_finger_linkage',
                             'right_finger_tip']

        pb.performCollisionDetection(self._client)
        contact_points = pb.getContactPoints(bodyA=self.robot_id,
                                             physicsClientId=self._client)

        for cont_pt in contact_points:
            robot_link = cont_pt[3]
            robot_link_name = self.link_names[robot_link]
            robot_link_name = self.link_names[robot_link]
            if robot_link_name not in ignored_links:
                other_body_id = cont_pt[2]
                other_body_name = pb.getBodyInfo(other_body_id,
                                                 physicsClientId=self._client
                                                )[1].decode('ascii')

                return True, {'robot_link' : robot_link_name,
                              'other_body' : other_body_name,
                              'other_link' : self.link_names[cont_pt[4]],
                             }

        return False, {}
