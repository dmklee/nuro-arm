import numpy as np
import numpy.random as npr

import pyfirmata as pyf
import time

POWER_CONTROL_PIN = 12
BIG_JOINT_MAXIMUM_SPEED = 140
DEFAULT_START_SPEED = 40
MS_PER_S=1000
SOFT_START_TIME=1000
HOME=(90, 90, 90, 90, 90, 73)
LOW=0
HIGH=1

class Braccio:
    def __init__(self, port='/dev/ttyACM0', baudrate=57600):
        self.board = pyf.Arduino(port, baudrate=baudrate)

        # make sure power isnt sent to servos
        self.power_control = self.board.get_pin('d:12:o')
        self.power_control.write(LOW)

        # assign pins for each servo
        self._joints = {
            'base'           : self.board.get_pin('d:11:s'),
            'shoulder'       : self.board.get_pin('d:10:s'),
            'elbow'          : self.board.get_pin('d:9:s'),
            'wristRotation'  : self.board.get_pin('d:5:s'),
            'wrist'          : self.board.get_pin('d:6:s'),
            'gripper'        : self.board.get_pin('d:3:s'),
        }
        for i,k in enumerate(self._joints.keys()):
            self._joints[k].write(HOME[i])
        self.power_control.write(HIGH)


    def _write_position(self, position):
        pass

    def move_to(self, position):
        pass

    def power_off(self):
        self.power_control.write(LOW)

    def power_on(self):
        self.power_control.write(HIGH)

    def soft_start(self):
        for _ in range(8):
            t = time.time()
            self.power_control.write(1)
            t = time.time()-t
            print(t)
            time.sleep(0.000001)
            self.power_control.write(0)
        self.power_control.write(1)

    def __del__(self):
        ''' power off and set positions to home '''
        self.power_off()
        for i,k in enumerate(self._joints.keys()):
            self._joints[k].write(HOME[i])


if __name__ == "__main__":
    b = Braccio()
    for _ in range(10):
        time.sleep(0.1)
    b.power_off()
