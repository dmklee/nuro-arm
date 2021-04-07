import numpy as np
import pybullet as pb
import pybullet_data

from abc import abstractmethod, ABC

# class BaseSimulator:
    # # creates pybullet simulator and implements basic functionality needed for 
    # # inverse/forward kinematics
    # def __init__():
        # # load urdf

    # def calculate_ik():
        # pass

    # def teleport_arm():
        # pass

    # def get_end_effector_pos():
        # pass

    # def get_end_effector_rot():
        # pass

    # def get_link_pos(link_name):
        # pass

# class BaseController(ABC):
    # def read_pos
    # def send_command

# class BaseRobot(ABC):
    # def __init__():
        # pass

    # def get_gripper_jpos():
        # pass

    # def get_arm_jpos():
        # pass

    # def move_joint(joint_name, pos):
        # pass

    # def close_gripper():
        # pass

    # def open_gripper():
        # pass

    # def move_gripper(jpos):
        # pass

    # def move_arm(jpos):
        # pass

    # def timestep(jpos):
        # # either time.sleep or pb.step
        # pass


# class SimulatedRobot():
# class xArmRobot():

class MotionPlanner():
    '''Use pybullet to handle IK, and collision checking'''
    URDF_FILE = "src/assets/xarm.urdf"
    def __init__(self, client=pb.DIRECT):
        self.client = pb.connect(client)

        # this path is where we find platform
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.loadURDF('plane.urdf', [0,0.5,0])

        # add xarm urdf
        self.id = pb.loadURDF(self.URDF_FILE, [0,0,0],[0,0,0,1],
                              flags=pb.URDF_USE_SELF_COLLISION)

        self.num_joints = pb.getNumJoints(self.id)
        self.end_effector_link_index = 5
        self.arm_joint_idxs = [0,1,2,3,4]
        self.gripper_joint_idxs = [6,7]

    def calculate_ik(self, pos, rot=None):
        if rot is not None and len(rot) == 3:
            rot = pb.getQuaternionFromEuler(rot)
        return pb.calculateInverseKinematics(self.id,
                                             self.end_effector_link_index,
                                             pos, rot)[:self.end_effector_link_index]

    def get_end_effector_pos(self, jpos):
        self._teleport_to_jpos(jpos)
        state = pb.getLinkState(self.id, self.end_effector_link_index)
        return state[4]

    def get_end_effector_rot(self, jpos):
        self._teleport_to_jpos(jpos)
        state = pb.getLinkState(self.id, self.end_effector_link_index)
        return pb.getEulerFromQuaternion(state[5])

    def _teleport_to_jpos(self, jpos):
        [pb.resetJointState(self.id, i, jp) for i,jp in enumerate(jpos)]

    def _get_gripper_jpos(self):
        return [pb.getJointState(self.id, idx)[0] for idx in self.gripper_joint_idxs]

    def _get_arm_jpos(self):
        return [pb.getJointState(self.id, idx)[0] for idx in self.arm_joint_idxs]

    def _command_gripper(self, jpos):
        pb.setJointMotorControlArray(self.id,
                                     self.gripper_joint_idxs,
                                     pb.POSITION_CONTROL,
                                     2*[jpos]
                                    )

    def _command_arm(self, jpos):
        assert len(jpos) == len(self.arm_joint_idxs)
        pb.setJointMotorControlArray(self.id,
                                     self.arm_joint_idxs,
                                     pb.POSITION_CONTROL,
                                     jpos
                                    )

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

class XArmBase(ABC):
    def __init__(self, mode):
        assert mode in ('robot', 'simulator')
        pb_client = {'robot' : pb.DIRECT,
                     'simulator' : pb.GUI,
                    }[mode]

        self._pb_sim = MotionPlanner(pb_client)

    def close_gripper(self) -> float:
        pass

    def open_gripper(self) -> float:
        pass

    def move_gripper(self, pos, rot=None) -> bool:
        pass

    def move_to_jpos(self, jpos) -> bool:
        """Move to a joint angle
        """
        pass

    def get_gripper_pos(self):
        pass

    def get_jpos(self):
        pass

class Robot(XArmBase):
    def gui(self):
        """Control each joint with GUI"""
        return

    def calibrate(self):
        """
        Do HOME position by setting offsets in hardware
        Handle any additional issues with software constraints on joint angles
        Save all this in config file
        Get gripper close/open positions
        """
        pass

class Simulator(XArmBase):
    GRIPPER_CLOSED = -0.3
    GRIPPER_OPENED = 0.3
    def __init__(self):
        super().__init__('simulator')

    def close_gripper(self) -> float:
        self._pb_sim._command_gripper(self.GRIPPER_CLOSED)
        return self._pb_sim._monitor_movement(
            self._pb_sim.gripper_joint_idxs,
            2*[self.GRIPPER_CLOSED],
            self._pb_sim._get_gripper_jpos
        )

    def open_gripper(self) -> float:
        self._pb_sim._command_gripper(self.GRIPPER_OPENED)
        return self._pb_sim._monitor_movement(
            self._pb_sim.gripper_joint_idxs,
            2*[self.GRIPPER_OPENED],
            self._pb_sim._get_gripper_jpos
        )

    def move_gripper(self, pos, rot=None) -> bool:
        jpos = self._pb_sim.calculate_ik(pos, rot)
        self.move_to_jpos(jpos)

    def move_to_jpos(self, jpos) -> bool:
        """Move to a joint angle
        """
        self._pb_sim._command_arm(jpos)
        return self._pb_sim._monitor_movement(
            self._pb_sim.arm_joint_idxs,
            jpos,
            self._pb_sim._get_arm_jpos
        )

    def get_gripper_pos(self):
        jpos = np.mean(self._pb_sim._get_gripper_jpos())
        return jpos

    def get_arm_jpos(self):
        return self._pb_sim._get_arm_jpos()

if __name__ == "__main__":
    import time
    robot = Simulator()
    while True:
        time.sleep(0.1)
        robot.open_gripper()
        time.sleep(0.1)
        robot.close_gripper()
        time.sleep(0.1)
        pos = np.random.uniform(-1,1,size=3)
        pos[2] = np.clip(pos[2], 0.1, 0.4)
        rot = np.random.uniform(0, np.pi, size=3)
        robot.move_gripper(pos, rot)
        time.sleep(1)
