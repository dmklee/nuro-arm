# import numpy as np
# import time
# import serial

# SIGNATURE = 0x55
# CMD_SERVO_MOVE = 0x03
# CMD_GET_BATTERY_VOLTAGE = 0x0f
# CMD_SERVO_STOP = 0x14
# CMD_GET_SERVO_POSITION  = 0x15

# def try_serial():
    # device = serial.Serial('/dev/ttyS0',
                           # baudrate=9600,
                           # parity=serial.PARITY_NONE,
                           # stopbits=serial.STOPBITS_ONE,
                           # bytesize=serial.EIGHTBITS,
                           # timeout=1)
    # device.flush()
    # device.write([SIGNATURE, SIGNATURE, 2, CMD_GET_BATTERY_VOLTAGE])
    # time.sleep(0.1)
    # data = device.read(4)
    # print(data)
    # if data[0] == SIGNATURE and data[1] == SIGNATURE and data[3] == CMD_GET_BATTERY_VOLTAGE:
        # length = data[2]
        # data = device.read(length)
    # print(data)
# if __name__ == "__main__":
    # vid=0x0483
    # pid=0x5750
    # import usb.core
    # import usb.util

    # dev = usb.core.find(idVendor=vid, idProduct=pid)

    # dev.write(

    # # intf = cfg[(0,0)]

    # # print(intf)
"""
sudo apt-get install libhidapi-hidraw0 libhidapi-libusb0
pip3 install --user hid
Notes:
- servos_off will power off the servos
- sending a movement command wakes up unpowered servos
- position readouts are sadly pretty slow, they can take up to 450ms
"""

import time
import easyhid
import numpy as np

def itos(v):
    lsb = v & 0xFF
    msb = v >> 8
    return lsb, msb

class XArm():
    def __init__(self, pid=22352):

        # Stores an enumeration of all the connected USB HID devices
        en = easyhid.Enumeration()
        for dev in en.find():
           print(dev.description())

        # return a list of devices based on the search parameters
        devices = en.find(vid=1155, pid=pid)

        # print a description of the devices found
        for dev in devices:
           print(dev.description())

        assert len(devices) > 0
        self.dev = devices[0]

        # open a device
        self.dev.open()
        print('Connected to xArm device')

    def __del__(self):
        print('Closing xArm device')
        self.dev.close()

    def move_to(self, id, pos, time=0):
        """
        CMD_SERVO_MOVE
        0x55 0x55 len 0x03 [time_lsb time_msb, id, pos_lsb pos_msb]
        Servo position is in range [0, 1000]
        """

        t_lsb, t_msb = itos(time)
        p_lsb, p_msb = itos(pos)
        self.dev.write([0x55, 0x55, 8, 0x03, 1, t_lsb, t_msb, id, p_lsb, p_msb])

    def move_all(self, poss, time=0):
        """
        Set the position of all servos at once
        """

        for i in range(6):
            self.move_to(id=i+1, pos=poss[i], time=time)

    def servos_off(self):
        self.dev.write([0x55, 0x55, 9, 20, 6, 1, 2, 3, 4, 5, 6])

    def read_pos(self):
        """
        Read the position of all 6 servos
        ServoPositionRead 21 (byte)count { (byte)id }; (byte)count { (byte)id (ushort)position }
        """

        self.dev.write([
            0x55, 0x55,
            9,  # Len
            21, # Cmd
            6,  # Count
            1,
            2,
            3,
            4,
            5,
            6
        ])

        ret = self.dev.read()
        count = ret[4]
        assert count == 6

        poss = []
        for i in range(6):
            id = ret[5 + 3*i]
            p_lsb = ret[5 + 3*i + 1]
            p_msb = ret[5 + 3*i + 2]
            pos = (p_msb << 8) + p_lsb
            poss.append(pos)

        return np.array(poss)

    def rest(self):
        self.move_all([500, 500, 200, 900, 800, 500], time=1500)
        time.sleep(2)
        self.servos_off()


class SafeXArm:
    """
    Wrapper to limit motion range and speed to maximize durability
    Also remaps joint angles into the [-1, 1] range
    """

    def __init__(self, **kwargs):
        self.arm = XArm(**kwargs)

        self.min_pos = np.array([
            100, # Base
            200,
            400,
            100,
            50,  # Wrist
            200, # Gripper
        ])

        self.max_pos = np.array([
            900, # Base
            800,
            900,
            600,
            850,  # Wrist
            650,  # Gripper
        ])

        # Maximum movement speed in (range/second)
        self.max_speed = 250

        self.move_all([0] * 6)
        time.sleep(2)

    def read_pos(self):
        return np.flip(self.arm.read_pos(), 0)

    def rest(self):
        return self.arm.rest()

    def move_all(self, pos):
        if not isinstance(pos, np.ndarray):
            pos = np.array(pos)

        # [-1, 1] => [0, 1]
        pos = (pos + 1) / 2
        target = self.min_pos + pos * (self.max_pos - self.min_pos)

        target = np.flip(target, 0).astype(np.uint16)

        # TODO: compute time needed based on last position
        # Compute time needed to move each joint to target given max_speed
        #cur_pos = self.arm.read_pos()
        #time = (abs(cur_pos - target) / self.max_speed)
        #time = (time * 1000).astype(np.uint16)

        for i in range(6):
            self.arm.move_to(id=i+1, pos=target[i], time=100)

def demo():
    arm = SafeXArm()

    # To the right
    arm.move_all([-1, 0, 0, 0, 0, 0])
    time.sleep(2)

    # To the left
    arm.move_all([1, 0, 0, 0, 0, 0])
    time.sleep(2)

    # Default position
    arm.move_all([0, 0, 0, 0, 0, 0])
    time.sleep(2)

    # Open gripper
    arm.move_all([0, 0, 0, 0, 0, -1])
    time.sleep(2)

    # Close gripper
    arm.move_all([0, 0, 0, 0, 0, 1])
    time.sleep(2)

    # Put the arm back in a resting position
    arm.rest()

demo()
