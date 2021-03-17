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
import threading

def itos(v):
    lsb = v & 0xFF
    msb = v >> 8
    return lsb, msb

class Servos(IntEnum):
    base = 6
    shoulder = 5
    elbow = 4
    wrist = 3
    wristRotation = 2
    gripper = 1

class xArmController():
    SIGNATURE = 85
    CMD_MOVE = 3
    CMD_POWER_OFF = 20
    CMD_POSITION_READ = 21
    CMD_POSITION_WRITE = 22
    CMD_OFFSET_READ = 23
    CMD_OFFSET_WRITE = 24

    LOWER_LIMIT = 0
    UPPER_LIMIT = 1000
    HOME = 500

    BITS2RADIANS = np.pi / 180. * ( 240. / 1000. )
    BITS_OFFSET = 500

    def __init__(self):
        self.device = self.connect()
        self.servos = Servos
        self.n_servos = len(self.servos)
        self._lock = threading.RLock()
        self.power_on()

    def connect(self):
        en = easyhid.Enumeration()
        devices = en.find(vid=1155, pid=22352)
        assert len(devices) == 1
        device = devices[0]
        device.open()
        print('Connected to xArm')
        return device

    def power_on(self):
        '''Turns on servos'''
        pos = self._read_all_servos_pos()
        self._move_all_servos(pos, duration=1000)

    def power_off(self):
        '''Turns off servos'''
        self._send(self.CMD_POWER_OFF, [6, 1,2,3,4,5,6])

    def disconnect(self):
        self.device.close()
        print('Disconnected xArm')

    def _home(self):
        home_pos = self.n_servos * [self.HOME]
        self._move_all_servos(home_pos, duration=1000)
        time.sleep(1)

    def _move_servo(self, servo_id, pos, duration=1000):
        data = [1, *itos(duration), servo_id, *itos(pos)]
        self._send(self.CMD_MOVE, data)

    def _move_all_servos(self, pos, duration=1000):
        assert len(pos) == self.n_servos
        for i,p in enumerate(pos):
            self._move_servo(i+1, p, duration)

    def _read_servo_pos(self, servo_id):
        self._send(self.CMD_POSITION_READ, [1, servo_id])
        pos = self._recv(self.CMD_POSITION_READ)[0]
        return pos

    def _read_all_servos_pos(self):
        self._send(self.CMD_POSITION_READ,
                   [self.n_servos, *range(1, self.n_servos+1)])
        all_pos = self._recv(self.CMD_POSITION_READ)
        # remember this is in order of the servo ids 1 -> 6
        return all_pos

    def _read_servo_offset(self, servo_id):
        self._send(self.CMD_OFFSET_READ, [1, servo_id])
        pos = self._recv(self.CMD_OFFSET_READ, ret_type='sbyte')[0]
        return pos

    def _write_servo_offset(self, servo_id, offset):
        offset = int(np.clip(offset, -127, 127))
        if offset < 0:
            offset = 255 + offset
        self._send(self.CMD_OFFSET_WRITE, [servo_id, offset])

    def _joint_pos_to_angle(self, joint_pos):
        return [(j-self.BITS_OFFSET)*self.BITS2RADIANS for j in joint_pos]

    def _joint_angle_to_pos(self, joint_angle):
        return [int(self.BITS_OFFSET+j/self.BITS2RADIANS) for j in joint_angle]

    def _send(self, cmd, data=[]):
        msg = bytearray([
            self.SIGNATURE,
            self.SIGNATURE,
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

    def __del__(self):
        '''Makes sure servos are off before disconnecting'''
        self.power_off()
        self.disconnect()

    def use_gui(self):
        def move_joint_fn_generator(servo_id):
            def foo(pos):
                pos = int(pos)
                print(f'moving to {pos}')
                self._move_servo(servo_id, pos, duration=1000)
            return foo

        def reset_servo_offsets():
            print('Changing servo offsets...')

            time.sleep(1)
            servo_pos = self._read_all_servos_pos()
            for servo in self.servos:
                old_offset = self._read_servo_offset(servo.value)
                true_HOME = self.HOME - old_offset

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
                                 from_=0, to=1000,
                                 orient=tk.HORIZONTAL,
                                 command=move_joint_fn)
            current_pos = self._read_servo_pos(servo_id)
            scl_joint.set(current_pos)
            scl_joint.pack()
            scales[servo_id] = scl_joint

        # Add button for changing servo offsets
        row_frame = tk.Frame(master=main_frame,
                             width=W,
                             height=H//7,
                             borderwidth=1)
        row_frame.pack(fill=tk.X)
        button_frame = tk.Frame(master=row_frame, width=W)
        button_frame.pack()

        btn = tk.Button(master=button_frame,
                        text='Reset HOME',
                        fg='red',
                        command=reset_servo_offsets)
        btn.pack()

        window.mainloop()

arm = xArmController()
arm.use_gui()
