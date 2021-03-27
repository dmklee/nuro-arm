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

#TODO:
    # error handling on receiving/sending
    # switch to passive servo-offset method
    # allow for calibration of joints using smart movements
    # create and write config file for storing offsets + servo directions

def itos(v):
    lsb = v & 0xFF
    msb = v >> 8
    return lsb, msb

class XArmController():
    SIGNATURE = 85
    CMD_MOVE = 3
    CMD_POWER_OFF = 20
    CMD_POSITION_READ = 21
    CMD_OFFSET_READ = 23
    CMD_OFFSET_WRITE = 24

    # in servo positional units
    SERVO_LOWER_LIMIT = 0
    SERVO_UPPER_LIMIT = 1000
    SERVO_HOME = 500
    SERVO_PRECISION = 10 # how accurate can we expect it to be
    SERVO_MAX_SPEED = 0.5 # positional units per millisecond

    POS2RADIANS = np.pi / 180. * ( 240. / 1000. )

    class Servos(IntEnum):
        base = 6
        shoulder = 5
        elbow = 4
        wrist = 3
        wristRotation = 2
        gripper = 1

    def __init__(self):
        self._jpos_home = self.SERVO_HOME*self.POS2RADIANS
        self._jpos_limits = (self._to_radians(self.SERVO_LOWER_LIMIT),
                            self._to_radians(self.SERVO_UPPER_LIMIT))
        self._max_speed = self.SERVO_MAX_SPEED*self.POS2RADIANS
        self._jpos_precision = self.SERVO_PRECISION * self.POS2RADIANS

        self.device = self.connect()
        self.servos = XArmController.Servos
        self.n_servos = len(self.servos)
        self._lock = threading.Lock()
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
        servo_ids = [s for s in self.servos]
        jpos = self.read_jpos(self.servos)
        self.move_jpos(jpos, servo_ids, duration=1000)

    def power_off(self):
        '''Turns off servos'''
        self._send(self.CMD_POWER_OFF, [6, 1,2,3,4,5,6])

    def disconnect(self):
        self.device.close()
        print('Disconnected xArm')

    def home(self):
        '''moves all servos to HOME_POS'''
        home_pos = self.n_servos * [self._jpos_home]
        self.move_servos(home_pos, self.servos, duration=1000)
        time.sleep(1)

    def _move_servo(self, jpos, servo_id, duration=1000):
        '''I have been unable to get multi-servo move command to work,
        so each servo must be commanded separately
        '''
        # prevent motion outside of servo limits
        jpos = np.clip(jpos, *self._jpos_limits)

        current_jpos = self.read_jpos([servo_id])[0]
        delta = abs(jpos - current_jpos)

        # ensure movement does not go above max speed
        duration = int(max(duration, delta / self._max_speed))

        # convert to positional units
        pos = self._to_pos_units(jpos)
        data = [1, *itos(duration), servo_id, *itos(pos)]
        self._send(self.CMD_MOVE, data)

        return duration

    def move_jpos(self, jpos, servo_ids, duration=1000):
        max_duration = 0
        for jp, s_id in zip(jpos, servo_ids):
            tmp_duration = self._move_servo(jp, s_id, duration)
            max_duration = max(max_duration, tmp_duration)
        return max_duration

    def safe_move_jpos(self, target_jpos, servo_ids, duration=1000,
                       monitor_freq=200, jpos_atol=None):
        '''Monitors motion to detect collisions. Returns success boolean

        :monitor_freq: how often position is checked in ms

        '''
        if jpos_atol is None:
            jpos_atol = self._jpos_precision
        exp_duration = self.move_jpos(target_jpos, servo_ids, duration)

        t_start = time.time()
        old_jpos = self.read_jpos(servo_ids)
        while time.time() - t_start < exp_duration:
            time.sleep(monitor_freq)
            new_jpos = self.read_jpos(servo_ids)

            # check if motion has stopped
            if np.allclose(old_jpos, new_jpos, atol=jpos_atol):
                if np.allclose(target_jpos, new_jpos, atol=jpos_atol):
                    return True
                break

        # target was not reached, set current jpos as target to prevent servos 
        # from overworking
        self.move_jpos(new_jpos, servo_ids, duration=0)
        return False

    def read_jpos(self, servo_ids):
        n_servos = len(servo_ids)
        self._send(self.CMD_POSITION_READ,
                   [n_servos, *servo_ids])
        pos = self._recv(self.CMD_POSITION_READ)

        # convert to radians
        jpos = [self._to_radians(p) for p in pos]
        return jpos

    def _read_servo_offset(self, servo_id):
        # returns in positional units
        self._send(self.CMD_OFFSET_READ, [1, servo_id])
        pos = self._recv(self.CMD_OFFSET_READ, ret_type='sbyte')[0]
        return pos

    def _write_servo_offset(self, servo_id, offset):
        # operates in in positional units
        offset = int(np.clip(offset, -127, 127))
        if offset < 0:
            offset = 255 + offset
        self._send(self.CMD_OFFSET_WRITE, [servo_id, offset])

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

    def _to_radians(self, pos):
        return (pos - self.SERVO_HOME) * self.POS2RADIANS

    def _to_pos_units(self, jpos):
        return int( jpos / self.POS2RADIANS + self.SERVO_HOME )

    def __del__(self):
        '''Makes sure servos are off before disconnecting'''
        self.power_off()
        self.disconnect()

    def get_pos_limits(self):
        return [(self.POS_LOWER_LIMIT, self.POS_UPPER_LIMIT) for _ in range(self.n_servos)]

    def use_gui(self):
        def move_joint_fn_generator(servo_id):
            def foo(jpos):
                jpos = float(jpos)
                self.move_jpos([jpos], [servo_id], duration=500)
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
            current_pos = self.read_jpos([servo_id])[0]
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

if __name__ == "__main__":
    arm = XArmController()
    arm.use_gui()
    # while True:
        # print([f"{a:0.2f}" for a in arm._read_all_servos_pos_angle()])
        # time.sleep(0.1)
    # arm.use_gui()
