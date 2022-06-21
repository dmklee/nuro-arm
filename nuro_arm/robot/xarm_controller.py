import os
import platform
import time
import numpy as np
import threading

from nuro_arm.robot.base_controller import BaseController
from nuro_arm.constants import XARM_CONFIG_FILE

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
    def __init__(self, serial_number=None):
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
            serial_number : str
                serial number of device
        '''
        if platform.system() == 'Linux':
            import easyhid
            self.exception = easyhid.easyhid.HIDException
            en = easyhid.Enumeration()
            devices = en.find(vid=1155, pid=22352, serial=serial_number)
            if len(devices) == 0:
                print('\n[ERROR] No device found. Ensure that the xarm is connected via '
                      'usb cable and the power is on.\n'
                      'Turn the xarm off and back on again if needed.\n')
                exit()
            elif len(devices) > 1:
                serial_numbers = ', '.join([f"\t{d.serial_number}" for d in devices])
                print('\n[ERROR] More than 1 xarm device found with the following serial numbers: \n'
                      f'    {serial_numbers}\n'
                      '  You must specify the serial number in this case.\n')
                exit()
            else:
                self.device = devices[0]
                self.serial_number = self.device.serial_number
                self.device.open()
                self.type = 0
        elif platform.system() == 'Windows':
            import hid
            if serial_number is not None:
                # serial number should be unicode
                serial_number = str(serial_number)

            try:
                self.device = hid.Device(vid=1155, pid=22352, serial=serial_number)
            except hid.HIDException:
                print('\n[ERROR] No device found. Ensure that the xarm is connected via '
                      'usb cable and the power is on.\n'
                      'Turn the xarm off and back on again if needed.\n')
                exit()

            self.serial_number = self.device.serial
            self.type = 1
        elif platform.system() == 'Darwin':
            import hid
            self.device = hid.device()
            try:
                if serial_number is not None:
                    # serial number should be unicode
                    serial_number = str(serial_number)

                self.device.open(1155, 22352, serial_number)
            except OSError:
                print('\n[ERROR] No device found. Ensure that the xarm is connected via '
                      'usb cable and the power is on.\n'
                      'Turn the xarm off and back on again if needed.\n')
                exit()

            self.serial_number = self.device.get_serial_number_string()
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
            #ToDo: why is this here, and when does this exception occur
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

    def __init__(self, serial_number=None):
        super().__init__()
        # speed in radians per second
        self.max_speed = 4.0
        self.min_speed = 0.1
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
        self.device, self.serial_number = self.connect(serial_number)

        if self.load_configs():
            self.power_on_servos()

            # write servo offsets, these get reset during power cycles
            for j_id in self.arm_joint_ids:
                new_offset = self.servo_offsets[j_id]
                old_offset = self._read_servo_offset(j_id)
                pos = self._to_pos_units(j_id, self.read_jpos([j_id])[0])
                self.move_servos([j_id], [self._to_radians(j_id, pos-new_offset+old_offset)])
                self._write_servo_offset(j_id, new_offset)

    def load_configs(self):
        if os.path.exists(XARM_CONFIG_FILE):
            data = np.load(XARM_CONFIG_FILE, allow_pickle=True).item()
            try:
                data = data[self.serial_number]
                self.arm_joint_directions = data['arm_joint_directions']
                self.gripper_joint_limits = data['gripper_joint_limits']
                self.servo_offsets = data['servo_offsets']
                return True
            except KeyError:
                pass

        print('\n[WARNING] Config file for this xarm could not be found. '
              ' Calibration should be performed.\n')
        self.arm_joint_directions = {i:1. for i in self.arm_joint_ids}
        self.gripper_joint_limits = np.array(((0.9,),(-1.,)))
        self.servo_offsets = {i:0 for i in self.servo_ids}
        return False

    def timestep(self):
        time.sleep(1/self.measurement_frequency)

    def connect(self, serial_number=None):
        device = Device(serial_number)
        serial_number = device.serial_number
        print(f'Connected to xArm (serial={serial_number})')
        return device, serial_number

    def disconnect(self):
        '''Closes HID connection to xArm
        '''
        self.device.close()
        print('Disconnected xArm')

    def power_on_servos(self):
        '''Turn on all servos so all joints are rigid
        '''
        current_jpos = self.read_jpos(self.servo_ids)
        self.write_jpos(self.servo_ids, current_jpos)

    def power_off_servos(self):
        '''Turn off all servos so all joints are passive
        '''
        [self.power_off_servo(i) for i in self.servo_ids]

    def power_on_servo(self, joint_id):
        '''Turn on single servo so the joint is rigid

        Parameters
        ----------
        joint_id : int
        '''
        current_jpos = self.read_jpos([joint_id])
        self.write_jpos([joint_id], current_jpos)

    def power_off_servo(self, joint_id):
        '''Turn off single servo so the joint is passive

        Parameters
        ----------
        joint_id : int
        '''
        self._send(CmdLib.POWER_OFF, [1, joint_id])

    def get_joint_id(self, joint_name):
        '''Get joint id associated with a given joint name, joint id may be used
        for writing/reading specific joint positions

        Parameters
        ----------
        joint_name : str, {'base', 'shoulder','elbow',
                           'wrist','wristRotation','gripper'}

        Returns
        -------
        int
            joint id
        '''
        return {'base' : 6,
                'shoulder' : 5,
                'elbow' : 4,
                'wrist' : 3,
                'wristRotation' : 2,
                'gripper' : 1
               }[joint_name]

    def get_joint_name(self, joint_id):
        '''Get joint name associated with a given joint id

        Parameters
        ----------
        joint_id : int

        Returns
        -------
        str
            joint name
        '''
        return {6 : 'base',
                5 : 'shoulder',
                4 : 'elbow',
                3 : 'wrist',
                2 : 'wristRotation',
                1 : 'gripper'
               }[joint_id]

    def write_jpos(self, joint_ids, jpos, speed=None):
        '''Issue move command to specified joint indices

        Parameters
        ----------
        joint_ids : array_like of int
            joint indices to be moved
        jpos : array_like of float
            target joint positions corresponding to the joint indices
        speed : float or array_like of float, default=None

        Returns
        -------
        float
            expected time (s) to complete movement
        '''
        if speed is None:
            speed = self.default_speed
        if np.isscalar(speed):
            speed = np.full(len(joint_ids), speed)

        speed = np.clip(speed, self.min_speed, self.max_speed)

        current_jpos = self.read_jpos(joint_ids)
        delta_jpos = np.abs(np.subtract(jpos, current_jpos))
        duration_ms = (1000 * delta_jpos / speed).astype(int)

        [self.move_servos([j_i], [j_p], dur)
             for j_i, j_p, dur in zip(joint_ids, jpos, duration_ms)]

        return np.max(duration_ms)/1000.

    def read_jpos(self, j_idxs):
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

        if len(pos) != len(j_idxs):
            print('\n[ERROR]: Unable to connect to all motors.'
                  ' Check that all wires between motors are connected properly.\n')
            del self
            exit()

        pos = np.clip(pos, self.SERVO_LOWER_LIMIT, self.SERVO_UPPER_LIMIT)

        jpos = [self._to_radians(i, p) for i,p in zip(j_idxs, pos)]
        return jpos

    def move_servos(self, joint_ids, jpos, duration=1000):
        # convert to positional units
        pos = [self._to_pos_units(j_id, jp) for j_id, jp in zip(joint_ids, jpos)]

        # ensure pos is within servo limits to prevent servo damage
        pos = np.clip(pos, self.SERVO_LOWER_LIMIT, self.SERVO_UPPER_LIMIT)

        data = [len(joint_ids), *itos(duration)]
        for j_id, p in zip(joint_ids, pos):
            data.extend([j_id, *itos(p)])

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
        '''Convert servo positional unit to angle in radians.  This will
        incorporate arm_joint_directions to reflect canonical joint angles
        (so it matches simulator)

        Parameters
        ----------
        joint_id : int
            joint id, must be in range [1,6]
        pos : int
            positional unit value, servos can report this in the range 0 to 1000

        Returns
        -------
        float
            joint position in radians
        '''
        jpos = (pos - self.SERVO_HOME) * self.POS2RADIANS
        if joint_id in self.arm_joint_ids:
            jpos *= self.arm_joint_directions[joint_id]

        return jpos

    def _to_pos_units(self, joint_id, jpos):
        '''Convert joint angle in radians to servo positional unit.  This will
        incorporate arm_joint_directions to reflect canonical joint angles
        (so it matches simulator)

        Does not clip positional unit value.

        Parameters
        ----------
        joint_id : int
            joint id, must be in range [1,6]
        jpos : float
            joint angle in radians

        Returns
        -------
        int
            servo positional unit value
        '''
        if joint_id in self.arm_joint_ids:
            jpos *= self.arm_joint_directions[joint_id]

        return int( jpos / self.POS2RADIANS + self.SERVO_HOME )

    def _reset_servo_offsets(self):
        '''Assumes robot is already in home position and servos are off
        '''
        success = True
        offsets = {}
        for j_id in self.arm_joint_ids:
            old_offset = self._read_servo_offset(j_id)
            true_home = self.SERVO_HOME - old_offset
            pos = self._to_pos_units(j_id, self.read_jpos([j_id])[0])
            new_offset = pos - true_home
            if abs(new_offset) > 127:
                success = False
            else:
                self.move_servos([j_id], [self._to_radians(j_id, pos-new_offset+old_offset)])
                self._write_servo_offset(j_id, new_offset)

            offsets[j_id] = new_offset
        return success, offsets

    def __del__(self):
        '''Makes sure servos are off before disconnecting
        '''
        try:
            self.power_off_servos()
            self.disconnect()
        except AttributeError:
            # device was never created so no need to disconnect
            pass

if __name__ == "__main__":
    xarm = XArmController()
