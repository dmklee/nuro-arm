import os
import platform
import time
import numpy as np
import threading

from nuro_arm.robot.base_controller import BaseController

def itos(v):
    lsb = v & 0xFF
    msb = v >> 8
    return lsb, msb

class CmdLib:
    SIGNATURE = 85
    MOVE = 3
    POWER_OFF = 20
    POSITION_READ = 21
    OFFSET_READ = 23
    OFFSET_WRITE = 24

class Device:
    def __init__(self):
        '''Abstraction of HID device so that the interface is the same across
        platforms

        Raises
        ------
        TypeError
            If operating system is not Darwin, Linux or Windows

        Attributes
        ----------
            device : obj
                hid device
            type : int
                indicates how to interact with hid device. if 1, send message as
                bytes and provide size argument to read; if 0, send message as
                bytearray without leading 0
        '''
        if platform.system() == 'Linux':
            import easyhid
            self.exception = easyhid.easyhid.HIDException
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
        '''Write message to device

        Parameters
        ----------
        msg : list
            list of data to be written
        '''
        try:
            if self.type == 0:
                self.device.write(bytearray(msg[1:]))
            elif self.type == 1:
                self.device.write(bytes(msg))
        except self.exception:
            pass

    def read(self, timeout):
        '''Read message from device

        Parameters
        ----------
        timeout : int
            timeout period in milliseconds

        Returns
        -------
        array_like
            message received from device
        '''
        if self.type == 0:
            return self.device.read(timeout)
        elif self.type == 1:
            size = 32
            return self.device.read(size, timeout)

    def close(self):
        '''Close hid device if possible
        '''
        if self.type == 0:
            self.device.close()

class XArmController(BaseController):
    # in servo positional units
    SERVO_LOWER_LIMIT = 0
    SERVO_UPPER_LIMIT = 1000
    SERVO_HOME = 500

    POS2RADIANS = np.pi / 180. * ( 240. / 1000. )

    CONFIG_FILE = "nuro_arm/robot/configs.npy"

    def __init__(self):
        super().__init__()
        # speed in radians per second
        self.max_speed = 2.0
        self.default_speed = 0.8

        self.arm_joint_ids = [6,5,4,3,2]
        self.gripper_joint_ids = [1]
        self.movement_precision = 0.04
        self.measurement_precision = 0.01
        self.measurement_frequency = 10

        self.arm_joint_limits = np.array(((-2, -1.58, -2, -1.8, -2),
                                          ( 2,  1.58,  2,  2.0,  2)))

        self.arm_jpos_home = np.zeros(len(self.arm_joint_ids))

        self.servo_ids = self.arm_joint_ids + self.gripper_joint_ids
        self.n_servos = len(self.servo_ids)
        self._lock = threading.Lock()
        self.device = self.connect()

        if self.load_configs():
            self.power_on_servos()

            # write servo offsets, these get reset during power cycles
            for j_id in self.arm_joint_ids:
                new_offset = self.servo_offsets[j_id]
                old_offset = self._read_servo_offset(j_id)
                pos = self._to_pos_units(j_id, self._read_jpos([j_id])[0])
                self._move_servo(j_id, self._to_radians(j_id, pos-new_offset+old_offset))
                self._write_servo_offset(j_id, new_offset)

    def load_configs(self):
        if os.path.exists(self.CONFIG_FILE):
            data = np.load(self.CONFIG_FILE, allow_pickle=True).item()
            try:
                self.arm_joint_directions = data.get('arm_joint_directions')
                self.gripper_joint_limits = data.get('gripper_joint_limits')
                self.servo_offsets = data.get('servo_offsets')
                return True
            except IndexError:
                pass

        print('[WARNING] xArm config file not found. '
              ' Calibration should be performed.')
        self.arm_joint_directions = {i:1. for i in self.arm_joint_ids}
        self.gripper_joint_limits = np.array(((0.9,),(-1.,)))
        self.servo_offsets = {i:0 for i in self.servo_ids}
        return False

    def timestep(self):
        time.sleep(1/self.measurement_frequency)

    def connect(self):
        device = Device()
        print('Connected to xArm')
        return device

    def disconnect(self):
        '''Closes HID connection to xArm
        '''
        self.device.close()
        print('Disconnected xArm')

    def power_on_servos(self):
        '''Turn on all servos so all joints are rigid
        '''
        current_jpos = self._read_jpos(self.servo_ids)
        self._write_jpos(self.servo_ids, current_jpos)

    def power_off_servos(self):
        '''Turn off all servos so all joints are passive
        '''
        [self.power_off_servo(i) for i in self.servo_ids]

    def power_on_servo(self, joint_id):
        '''Turn on single servo so the joint is rigid
        '''
        current_jpos = self._read_jpos([joint_id])
        self._write_jpos([joint_id], current_jpos)

    def power_off_servo(self, joint_id):
        '''Turn off single servo so the joint is passive
        '''
        self._send(CmdLib.POWER_OFF, [1, joint_id])

    def get_joint_id(self, joint_name):
        return {'base' : 6,
                'shoulder' : 5,
                'elbow' : 4,
                'wrist' : 3,
                'wristRotation' : 2,
                'gripper' : 1
               }[joint_name]

    def get_joint_name(self, joint_id):
        return {6 : 'base',
                5 : 'shoulder',
                4 : 'elbow',
                3 : 'wrist',
                2 : 'wristRotation',
                1 : 'gripper'
               }[joint_id]

    def _write_jpos(self, joint_ids, jpos, speed=None):
        '''Issue move command to specified joint indices

        Parameters
        ----------
        joint_ids : array_like of int
            joint indices to be moved
        jpos : array_like of float
            target joint positions corresponding to the joint indices
        speed : float

        Returns
        -------
        float
            expected time (s) to complete movement
        '''
        if speed is None:
            speed = self.default_speed

        # we need to ensure linear motion without violating max speed
        current_jpos = self._read_jpos(joint_ids)
        delta_jpos = np.abs(np.subtract(jpos, current_jpos))

        # milliseconds
        duration = int(1000 * np.max(delta_jpos) / speed)

        [self._move_servo(j_i, j_p, duration)
             for j_i, j_p in zip(joint_ids, jpos)]

        return duration/1000

    def _read_jpos(self, j_idxs):
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
        self._send(CmdLib.POSITION_READ,
                   [len(j_idxs), *j_idxs])
        pos = self._recv(CmdLib.POSITION_READ, ret_type='short')

        # ensure no readings are outside range
        pos = np.clip(pos, self.SERVO_LOWER_LIMIT, self.SERVO_UPPER_LIMIT)

        # convert to radians
        jpos = [self._to_radians(i, p) for i,p in zip(j_idxs, pos)]
        return jpos

    def _move_servo(self, joint_id, jpos, duration=1000):
        '''I have been unable to get multi-servo move command to work,
        so each servo must be commanded separately
        '''
        #TODO: what is the maximum allowable duration

        # convert to positional units
        pos = self._to_pos_units(joint_id, jpos)

        # ensure pos is within servo limits to prevent servo damage
        pos = np.clip(pos, self.SERVO_LOWER_LIMIT, self.SERVO_UPPER_LIMIT)

        data = [1, *itos(duration), joint_id, *itos(pos)]
        self._send(CmdLib.MOVE, data)

    def _read_servo_offset(self, servo_id):
        # returns in positional units
        self._send(CmdLib.OFFSET_READ, [1, servo_id])
        pos = self._recv(CmdLib.OFFSET_READ, ret_type='char')[0]
        return pos

    def _write_servo_offset(self, servo_id, offset):
        # operates in in positional units
        offset = int(np.clip(offset, -127, 127))
        if offset < 0:
            offset = 255 + offset
        self._send(CmdLib.OFFSET_WRITE, [servo_id, offset])

    def _send(self, cmd, data=[]):
        msg = [
            0, # this will be deleted for easyhid
            CmdLib.SIGNATURE,
            CmdLib.SIGNATURE,
            len(data)+2,
            cmd,
            *data
        ]
        with self._lock:
            self.device.write(msg)

    def _recv(self, cmd, ret_type, timeout=1000):
        '''
        short : -32,768 to 32,767 for positional readings
        char : -128 to 127 for servo offsets
        '''
        with self._lock:
            ret = self.device.read(timeout)
        if ret is None:
            # timed out
            return ret
        count = ret[4]

        assert ret[3] == cmd

        recv_data = []
        packet_size = 3 if ret_type == 'short' else 2
        for i in range(count):
            lsb = ret[5 + packet_size*i + 1]
            msb = ret[5 + packet_size*i + 2]
            if ret_type == 'short':
                data = (msb << 8) + lsb
                if data > 32767:
                    data = data - 65_535
            elif ret_type == 'char':
                data = lsb if lsb < 128 else lsb-255
            else:
                raise TypeError
            recv_data.append(data)
        return recv_data

    def _to_radians(self, joint_id, pos):
        jpos = (pos - self.SERVO_HOME) * self.POS2RADIANS
        if joint_id in self.arm_joint_ids:
            jpos *= self.arm_joint_directions[joint_id]

        return jpos

    def _to_pos_units(self, joint_id, jpos):
        if joint_id in self.arm_joint_ids:
            jpos *= self.arm_joint_directions[joint_id]

        return int( jpos / self.POS2RADIANS + self.SERVO_HOME )

    def _reset_servo_offsets(self):
        """Assumes robot is already in home position and servos are off"""
        success = True
        offsets = {}
        for j_id in self.arm_joint_ids:
            old_offset = self._read_servo_offset(j_id)
            true_home = self.SERVO_HOME - old_offset
            pos = self._to_pos_units(j_id, self._read_jpos([j_id])[0])
            new_offset = pos - true_home
            if abs(new_offset) > 127:
                success = False
            else:
                self._move_servo(j_id, self._to_radians(j_id, pos-new_offset+old_offset))
                self._write_servo_offset(j_id, new_offset)

            offsets[j_id] = new_offset
        return success, offsets

    def __del__(self):
        '''Makes sure servos are off before disconnecting'''
        self.power_off_servos()
        self.disconnect()

