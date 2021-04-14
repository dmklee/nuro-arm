import time
from enum import IntEnum
import platform
if platform.system() == 'Linux':
    import easyhid
elif platform.system() == 'Windows':
    import serial
elif platform.system() == 'Darwin':
    pass
import numpy as np
import cv2
import threading
from src.base_controller import BaseController
import matplotlib.pyplot as plt

#TODO:
    # error handling on receiving/sending
    # switch to passive servo-offset method
    # allow for calibration of joints using smart movements
    # create and write config file for storing offsets + servo directions

def itos(v):
    lsb = v & 0xFF
    msb = v >> 8
    return lsb, msb

class XArmController(BaseController):
    # in servo positional units
    SERVO_LOWER_LIMIT = 0
    SERVO_UPPER_LIMIT = 1000
    SERVO_HOME = 500
    SERVO_PRECISION = 10 # how accurate can we expect it to be
    SERVO_MAX_SPEED = 0.5 # positional units per millisecond

    POS2RADIANS = np.pi / 180. * ( 240. / 1000. )

    CONFIG_FILE = "src/configs/robot.npz"

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
        self._jpos_home = self._to_radians(self.SERVO_HOME)
        self._jpos_limits = (self._to_radians(self.SERVO_LOWER_LIMIT),
                            self._to_radians(self.SERVO_UPPER_LIMIT))
        self._max_speed = self.SERVO_MAX_SPEED*self.POS2RADIANS
        self._jpos_precision = self.SERVO_PRECISION * self.POS2RADIANS

        self.servos = XArmController.Servos
        self.n_servos = len(self.servos)
        self._lock = threading.Lock()
        self.device = self.connect()
        self.power_on()

        self.arm_joint_idxs = [6,5,4,3,2]
        self.gripper_joint_idxs = [1]
        self.gripper_closed
        self.gripper_opened

    def connect(self):
        en = easyhid.Enumeration()
        devices = en.find(vid=1155, pid=22352)
        assert len(devices) == 1
        device = devices[0]
        device.open()
        print('Connected to xArm')
        return device

    def power_on(self):
        '''Turns on servos. sets position command to current position'''
        jpos = self.read_command(self.servos)
        self.move_command(self.servos, jpos)

    def power_off(self):
        '''Turns off servos'''
        self._send(self.cmd_lib.POWER_OFF, [6, 1,2,3,4,5,6])

    def disconnect(self):
        self.device.close()
        print('Disconnected xArm')

    def home(self):
        '''moves all servos to HOME_POS'''
        home_pos = self.n_servos * [self._jpos_home]
        self.move_command(self.servos, home_pos, duration=1000)
        time.sleep(1)

    def move_command(self, j_idxs, jpos):
        [self._move_servo(j_p, j_id) for j_p, j_id in zip(jpos, j_idxs)]

    def read_command(self, j_idxs):
        self._send(self.cmd_lib.POSITION_READ,
                   [len(j_idxs), *j_idxs])
        pos = self._recv(self.cmd_lib.POSITION_READ)

        # convert to radians
        j_pos = [self._to_radians(p) for p in pos]
        return j_pos

    def _move_servo(self, jpos, servo_id, duration=1000):
        '''I have been unable to get multi-servo move command to work,
        so each servo must be commanded separately
        '''
        # prevent motion outside of servo limits
        jpos = np.clip(jpos, *self._jpos_limits)

        current_jpos = self.read_command([servo_id])[0]
        delta = abs(jpos - current_jpos)

        # ensure movement does not go above max speed
        duration = int(max(duration, delta / self._max_speed))

        # convert to positional units
        pos = self._to_pos_units(jpos)
        data = [1, *itos(duration), servo_id, *itos(pos)]
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
        msg = bytearray([
            self.cmd_lib.SIGNATURE,
            self.cmd_lib.SIGNATURE,
            len(data)+2,
            cmd,
            *data
        ])
        with self._lock:
            self.device.write(msg)

    def _recv(self, cmd, ret_type='ushort', timeout=1000):
        assert ret_type in ('ushort', 'byte', 'sbyte')
        with self._lock:
            ret = self.device.read(timeout=timeout)

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

    def _to_radians(self, pos):
        return (pos - self.SERVO_HOME) * self.POS2RADIANS

    def _to_pos_units(self, jpos):
        return int( jpos / self.POS2RADIANS + self.SERVO_HOME )

    def __del__(self):
        '''Makes sure servos are off before disconnecting'''
        self.power_off()
        self.disconnect()

    def _reset_servo_offsets(self):
        """Assumes robot is already in home position and servos are off"""
        offsets = {}
        for servo in self.servos:
            if servo.name == 'gripper':
                continue
            old_offset = self._read_servo_offset(servo)
            true_home = self.SERVO_HOME - old_offset
            pos = self._to_pos_units(self.read_jpos([servo])[0])
            new_offset = pos - true_home
            self._write_servo_offset(servo.value, new_offset)
            offsets[f"{servo.name}_offset"] = new_offset
            # print(f'  {servo.name} offset set to {new_offset} servo units')
        return offsets

    def use_gui(self):
        def move_joint_fn_generator(servo_id):
            def foo(jpos):
                jpos = float(jpos)
                self.move_command([jpos], [servo_id], duration=500)
            return foo

        def reset_servo_offsets():
            print('not supported currently')
            return
            print('Changing servo offsets...')

            time.sleep(1)
            servo_pos = self._read_all_servos_pos()
            for servo in self.servos:
                old_offset = self._read_servo_offset(servo.value)
                true_HOME = self.POS_HOME - old_offset

                pos = scales[servo.value].get()
                new_offset = pos - true_HOME

                #preemptively send position command, otherwise the servo moves
                # too rapidly when the offset is changed suddenly
                self._move_servo(servo.value, pos-new_offset+old_offset)
                self._write_servo_offset(servo.value, new_offset)

                new_offset = self._read_servo_offset(servo.value)
                print(f'\t{servo.name} offset changed from {old_offset} to {new_offset}')
                new_pos = self._read_servo_pos(servo.value)
                scales[servo.value].set(new_pos)

            print('...moving to new home position')
            time.sleep(1)
            return

        H,W = 600, 400
        import tkinter as tk
        window = tk.Tk()
        heading = tk.Label(text="xArm GUI")
        heading.pack()

        main_frame = tk.Frame(master=window, width=W, height=H)
        main_frame.pack(fill=tk.BOTH)

        # reverse order so base is at bottom
        scales = dict()
        for servo in reversed(self.servos):
            servo_id = servo.value
            servo_name = servo.name
            row_frame = tk.Frame(master=main_frame, width=W, height=H//7, borderwidth=1)
            row_frame.pack(fill=tk.X)

            col_frame_left = tk.Frame(master=row_frame, width=W//2)
            col_frame_left.pack(side=tk.LEFT)
            col_frame_right = tk.Frame(master=row_frame, width=W//2)
            col_frame_right.pack(side=tk.RIGHT)

            lbl_joint = tk.Label(master=col_frame_left, text=servo_name)
            lbl_joint.pack()

            move_joint_fn = move_joint_fn_generator(servo_id)
            scl_joint = tk.Scale(master=col_frame_right,
                                 from_=self._jpos_limits[0],
                                 to=self._jpos_limits[1],
                                 resolution=self._jpos_precision,
                                 orient=tk.HORIZONTAL,
                                 command=move_joint_fn)
            current_pos = self.read_command([servo_id])[0]
            scl_joint.set(current_pos)
            scl_joint.pack()
            scales[servo_id] = scl_joint

        # # Add button for changing servo offsets
        # row_frame = tk.Frame(master=main_frame,
                             # width=W,
                             # height=H//7,
                             # borderwidth=1)
        # row_frame.pack(fill=tk.X)
        # button_frame = tk.Frame(master=row_frame, width=W)
        # button_frame.pack()

        # btn = tk.Button(master=button_frame,
                        # text='Reset HOME',
                        # fg='red',
                        # command=reset_servo_offsets)
        # btn.pack()

        window.mainloop()

    def arm_calibration(self):
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

        # calibrate motor directions
        print('WARNING: motor directions are not yet calibrated')
        return True, data

    def gripper_calibration(self):
        data = {}
        print('  =========================  ')
        print('  == Calibrating gripper ==  ')
        print('  =========================  ')
        print('Move gripper to fully closed position.')
        if input('   ready? [y/n]: ') != 'y':
            print('Calibration terminated.')
            return False, data
        gripper_closed = self.read_command(self.gripper_joint_idxs)[0]
        data.update({'gripper_closed' : gripper_closed})
        print(f"  gripper closed position is {gripper_closed:.2f} radians.")

        print('Move gripper to fully opened position.')
        if input('   ready? [y/n]: ') != 'y':
            print('Calibration terminated.')
            return False, data
        gripper_opened = self.read_command(self.gripper_joint_idxs)[0]
        data.update({'gripper_opened' : gripper_opened})
        print(f"  gripper opened position is {gripper_opened:.2f} radians.")
        return True, data

    def full_calibration(self):
        # passive mode
        self.power_off()

        data = {}
        ret, new_data = self.arm_calibration()
        if ret:
            data.update(new_data)

        ret, new_data = self.gripper_calibration()
        if ret:
            data.update(new_data)

        np.savez(self.CONFIG_FILE, **data)

        self.power_on()
        return True, data

if __name__ == "__main__":
    arm = XArmController()
    arm.calibrate()
    # arm.use_gui()
    # while True:
        # print([f"{a:0.2f}" for a in arm._read_all_servos_pos_angle()])
        # time.sleep(0.1)
    # arm.use_gui()
