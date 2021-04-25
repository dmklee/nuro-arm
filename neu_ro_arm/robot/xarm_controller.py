import time
import ctypes
from enum import IntEnum
import platform
import numpy as np
import cv2

import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt

import threading
from neu_ro_arm.robot.base_controller import BaseController

class InvalidServoOffset(Exception):
    def __init__(self, j_idx):
        message = f"Servo offset is too large for motor id={j_idx}. Servo must be re-installed."
        super().__init__(message)

def itos(v):
    lsb = v & 0xFF
    msb = v >> 8
    return lsb, msb

class Device:
    def __init__(self):
        if platform.system() == 'Linux':
            import easyhid
            en = easyhid.Enumeration()
            devices = en.find(vid=1155, pid=22352)
            assert len(devices) == 1
            self.device = devices[0]
            self.device.open()
            self.type = 0
        elif platform.system() == 'Windows':
            import hid
            self.device = hid.Device(vid=1155, pid=22352)
            self.type = 1
        elif platform.system() == 'Darwin':
            import hid
            self.device = hid.device()
            self.device.open(1155, 22352)
            self.type = 0
        else:
            raise TypeError('unsupported operating system')

    def write(self, msg):
        if self.type == 0:
            self.device.write(bytearray(msg[1:]))
        elif self.type == 1:
            self.device.write(bytes(msg))

    def read(self, timeout):
        if self.type == 0:
            return self.device.read(timeout)
        elif self.type == 1:
            size = 32
            return self.device.read(size, timeout)

    def close(self):
        if self.type == 0:
            self.device.close()

class XArmController(BaseController):
    # in servo positional units
    SERVO_LOWER_LIMIT = 0
    SERVO_UPPER_LIMIT = 1000
    SERVO_HOME = 500
    SERVO_PRECISION = 10 # how accurate can we expect it to be
    SERVO_MAX_SPEED = 0.5 # positional units per millisecond

    POS2RADIANS = np.pi / 180. * ( 240. / 1000. )

    CONFIG_FILE = "neu_ro_arm/robot/configs.npz"

    class CommandLibrary:
        SIGNATURE = 85
        MOVE = 3
        POWER_OFF = 20
        POSITION_READ = 21
        OFFSET_READ = 23
        OFFSET_WRITE = 24

    class Servos(IntEnum):
        base = 6
        shoulder = 5
        elbow = 4
        wrist = 3
        wristRotation = 2
        gripper = 1

    def __init__(self):
        self.cmd_lib = XArmController.CommandLibrary()
        self._max_speed = self.SERVO_MAX_SPEED*self.POS2RADIANS

        self.arm_joint_idxs = [6,5,4,3,2]
        self.gripper_joint_idxs = [1]
        self.joint_precision = self.SERVO_PRECISION * self.POS2RADIANS

        # load params from config file
        configs = np.load(self.CONFIG_FILE)
        self.gripper_closed = configs.get('gripper_closed', None)
        self.gripper_opened = configs.get('gripper_opened', None)
        self.arm_motor_directions = configs.get('arm_motor_directions', None)
        self.arm_jpos_home = np.full(len(self.arm_joint_idxs),
                                     self._to_radians(self.SERVO_HOME)
                                    )
        self.joint_limits = { 1 : (-np.pi/2, np.pi/2),
                              2 : (-np.pi/2, np.pi/2),
                              3 : (-np.pi/2, np.pi/2),
                              4 : (-np.pi/2, np.pi/2),
                              5 : (-np.pi/2, np.pi/2),
                              6 : (-np.pi/2, np.pi/2)
                            }

        self.servos = XArmController.Servos
        self.n_servos = len(self.servos)
        self._lock = threading.Lock()
        self.device = self.connect()
        # self.power_on()

    def connect(self):
        device = Device()
        print('Connected to xArm')
        return device

    def power_off(self):
        '''Powers off servos, used to enable passive mode or before disconnecting
        '''
        self._send(self.cmd_lib.POWER_OFF, [6, 1,2,3,4,5,6])

    def disconnect(self):
        '''Closes HID connection to xArm
        '''
        self.device.close()
        print('Disconnected xArm')

    def move_command(self, j_idxs, jpos):
        '''Issue move command to specified joint indices

        This simulator runs realtime and I have not tried to mimic the movement
        speed of the real robot.  The movements are meant to be linear in joint
        space to reflect movements of xArm.

        Parameters
        ----------
        j_idxs : array_like of int
            joint indices to be moved
        jpos : array_like of float
            target joint positions corresponding to the joint indices
        '''
        [self._move_servo(j_p, j_id) for j_p, j_id in zip(jpos, j_idxs)]

    def read_command(self, j_idxs):
        '''Read some joint positions

        Parameters
        ----------
        j_idxs : array_like of int
            joint indices whose position should be read

        Returns
        -------
        jpos : list of float
            joint positions in radians, will be same length as j_idxs
        '''
        self._send(self.cmd_lib.POSITION_READ,
                   [len(j_idxs), *j_idxs])
        pos = self._recv(self.cmd_lib.POSITION_READ)

        # convert to radians
        j_pos = [self._to_radians(p) for p in pos]
        return j_pos

    def _move_servo(self, jpos, j_idx, duration=1000):
        '''I have been unable to get multi-servo move command to work,
        so each servo must be commanded separately
        '''
        # prevent motion outside of servo limits
        jpos = np.clip(jpos, *self.joint_limits[j_idx])

        current_jpos = self.read_command([j_idx])[0]
        delta = abs(jpos - current_jpos)

        # ensure movement does not go above max speed
        duration = int(max(duration, delta / self._max_speed))

        # convert to positional units
        pos = self._to_pos_units(j_idx, jpos)
        data = [1, *itos(duration), j_idx, *itos(pos)]
        self._send(self.cmd_lib.MOVE, data)

        return duration

    def _read_servo_offset(self, servo_id):
        # returns in positional units
        self._send(self.cmd_lib.OFFSET_READ, [1, servo_id])
        pos = self._recv(self.cmd_lib.OFFSET_READ, ret_type='sbyte')[0]
        # print('warning: _read_servo_offset returns servo units not radians')
        return pos

    def _write_servo_offset(self, servo_id, offset):
        # operates in in positional units
        offset = int(np.clip(offset, -127, 127))
        if offset < 0:
            offset = 255 + offset
        self._send(self.cmd_lib.OFFSET_WRITE, [servo_id, offset])

    def _send(self, cmd, data=[]):
        msg = [
            0, # this will be deleted for easyhid
            self.cmd_lib.SIGNATURE,
            self.cmd_lib.SIGNATURE,
            len(data)+2,
            cmd,
            *data
        ]
        with self._lock:
            self.device.write(msg)

    def _recv(self, cmd, ret_type='ushort', timeout=1000):
        assert ret_type in ('ushort', 'byte', 'sbyte')
        with self._lock:
            ret = self.device.read(timeout)
        if ret is None:
            # timed out
            return ret
        count = ret[4]

        assert ret[3] == cmd

        recv_data = []
        packet_size = 3 if ret_type == 'ushort' else 2
        for i in range(count):
            servo_id = ret[5 + packet_size*i]
            lsb = ret[5 + packet_size*i + 1]
            msb = ret[5 + packet_size*i + 2]
            if ret_type == 'ushort':
                data = (msb << 8) + lsb
            elif ret_type == 'byte':
                data = lsb
            else: # 'sbyte'
                data = lsb if lsb < 128 else lsb-255
            recv_data.append(data)
        return recv_data

    def _to_radians(self, idx, pos):
        jpos = (pos - self.SERVO_HOME) * self.POS2RADIANS
        if idx in self.arm_joint_idxs:
            jpos *= self.arm_motor_directions[idx]

        return jpos

    def _to_pos_units(self, idx, jpos):
        if idx in self.arm_joint_idxs:
            jpos *= self.arm_motor_directions[idx]

        return int( jpos / self.POS2RADIANS + self.SERVO_HOME )

    def __del__(self):
        '''Makes sure servos are off before disconnecting'''
        # print('xArm shutting down, returning to home position momentarily...')
        # time.sleep(3)
        # self.home()
        self.power_off()
        self.disconnect()

    def _reset_servo_offsets(self):
        """Assumes robot is already in home position and servos are off"""
        offsets = {}
        # for servo in self.servos:
        for j_idx in self.arm_joint_idxs:
            old_offset = self._read_servo_offset(j_idx)
            true_home = self.SERVO_HOME - old_offset
            pos = self._to_pos_units(self.read_command([j_idx])[0])
            new_offset = pos - true_home
            if abs(new_offset) > 127:
                raise InvalidServoOffset(j_idx)

            self._move_servo(self._to_radians(pos-new_offset+old_offset),
                             j_idx)
            self._write_servo_offset(j_idx, new_offset)
            offsets[f"offset_j{j_idx}"] = new_offset
        return offsets

    def calibrate_arm(self):
        self.power_off()
        data = {}
        print('  =====================  ')
        print('  == Calibrating arm ==  ')
        print('  =====================  ')
        print('Please move arm into home position.')
        inp = input('  ready? [y/n/picture]: ')
        if inp == 'y':
            pass
        elif inp == 'picture':
            img = np.concatenate((cv2.imread('data/arm_home_position_front.jpg'),
                                  cv2.imread('data/arm_home_position_side.jpg')),
                                axis=1
                                )
            plt.figure()
            plt.imshow(img)
            plt.axis('off')
            plt.title('Move arm into this position (gripper position can be ignored)')
            plt.show()
            if input('  ready? [y/n]: ') != 'y':
                print('Calibration terminated')
                return False, data
        else:
            print('Calibration terminated.')
            return False, data

        print('performing servo offset correction...')
        arm_servo_offsets = self._reset_servo_offsets()
        data.update(arm_servo_offsets)
        print('finished servo offset correction.')

        print()
        print('Checking motor directions...')
        print('Please move robot into the configuration shown in the picture.')
        self.power_off()
        img = cv2.imread('data/arm_motor_calibration.jpg')
        plt.figure()
        plt.imshow(img)
        plt.axis('off')
        plt.title('Move arm into this approximate position')
        plt.show()
        if input('  ready? [y/n]: ') != 'y':
            print('Calibration terminated')
            return False, data
        arm_jpos = self.read_command(self.arm_joint_idxs)
        data['arm_motor_directions'] = np.sign(arm_jpos)

        self.power_on()
        return True, data

    def calibrate_gripper(self):
        self.power_off()
        data = {}
        print('  =========================  ')
        print('  == Calibrating gripper ==  ')
        print('  =========================  ')
        print('Move gripper to fully closed position.')
        if input('   ready? [y/n]: ') != 'y':
            print('Calibration terminated.')
            return False, data
        gripper_closed = self.read_command(self.gripper_joint_idxs)
        data.update({'gripper_closed' : np.array(gripper_closed)})
        # print(f"  gripper closed position is {gripper_closed[0]:.2f} radians.")
        print()

        print('Move gripper to fully opened position.')
        if input('   ready? [y/n]: ') != 'y':
            print('Calibration terminated.')
            return False, data
        gripper_opened = self.read_command(self.gripper_joint_idxs)
        data.update({'gripper_opened' : np.array(gripper_opened)})
        # print(f"  gripper opened position is {gripper_opened[0]:.2f} radians.")
        self.power_on()
        return True, data

    def calibrate(self):
        data = {}
        ret, new_data = self.calibrate_arm()
        if ret:
            data.update(new_data)
        else:
            return False, None

        ret, new_data = self.calibrate_gripper()
        if ret:
            data.update(new_data)
        else:
            return False, None


        return True, data

if __name__ == "__main__":
    arm = XArmController()
    # arm.power_off()
    names = ['base', 'shoulder', 'elbow', 'wrist', 'wristRotation']
    while True:
        jpos = arm.read_command(arm.arm_joint_idxs)
        print([f"{n}:{jp:.2f}" for jp,n in zip(jpos, names)])
        # pos = [arm._to_pos_units(jp) for jp in jpos]
        # print([f"{n}:{p}" for p,n in zip(pos, names)])

        # print([f"{a:0.2f}" for a in arm._read_all_servos_pos_angle()])
        time.sleep(0.1)
