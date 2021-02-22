import numpy as np
import serial
import time
import os

from robots.joint_state import JointStates


class Braccio:
    HOME = np.array((1.5708, 1.5708, 1.5708, 1.5708, 1.5708, 1.2740, 1.2740))
    URDF_PATH = 'robots/urdf/braccio.urdf'
    # taken from https://github.com/arduino-libraries/Braccio/blob/master/src/Braccio.cpp
    JOINT_LIMITS = np.array(((0.      , 3.1416),
                             (0.2618  , 2.8798),
                             (0.      , 3.1416),
                             (0.      , 3.1416),
                             (0.      , 3.1416),
                             (0.1745  , 1.2740),
                             (0.1745  , 1.2740)))
    #TODO: handle erros over serial port
    def __init__(self, port_name='/dev/ttyACM0'):
        self.move_speed = 100
        self.port = None
        self.connect_serial(port_name)

        urdf_path = os.path.join(os.getcwd(), self.URDF_PATH)
        self.joint_states = JointStates(self.HOME, self.JOINT_LIMITS, self.URDF_PATH)

    def connect_serial(self, port_name):
        print("Setting up serial port connection on {}...".format(port_name))
        try:
            self.port = serial.Serial(port_name, 115200, timeout=5)
            time.sleep(3)
        except (FileNotFoundError, serial.serialutil.SerialException) as e:
            print("[ERROR] Connection could not be established. "
                  "Make sure cable is properly connected and try again.")
            exit()
        else:
            print("Connection established. Make sure robot is in home position"
                  " before plugging in power cable.")

    def open_gripper(self):
        self.joint_states.open_gripper()
        self.go(self.joint_states)

    def close_gripper(self):
        self.joint_states.close_gripper()
        self.go(self.joint_states)

    def go(self):
        angles_deg = self.joint_states.to_degrees(round=True)
        command_string = ('P{:d},{:d},{:d},{:d},{:d},{:d}').format(
            *angles_deg)
        command_string += ',{:d}\n'.format(self.move_speed)
        self._write_serial(command_string)

    def set_gripper_position(self, position):
        self.joint_states.set_gripper_position(position)

    def get_gripper_position(self):
        return self.joint_states.get_gripper_position()

    def set_joint_angle(self, joint_name, angle):
        self.joint_states[joint_name] = angle

    def get_joint_angle(self, joint_name):
        return self.joint_states[joint_name]

    def go_home(self):
        self._write_serial('H\n')

    def power_on(self):
        self._write_serial('1\n')

    def power_off(self):
        if self.port is not None:
            self._write_serial('0\n')

    def __del__(self):
        self.go_home()
        self.power_off()

    def _write_serial(self, string):
        self.port.write(string.encode())
        # print(self.port.readline().decode())

if __name__ == "__main__":
    robot = Braccio()
    joint_name = 'base'
    delta = 0.1
    angle = robot.get_joint_angle(joint_name)
    print(angle)
    for _ in range(5):
        # robot.open_gripper()
        robot.set_joint_angle(joint_name, angle + delta)
        robot.go()
        angle = robot.get_joint_angle(joint_name)
        # print(angle)
        print(robot.get_gripper_position())
        print(robot.joint_states.angles())
        time.sleep(1)

# to do:
    # handle serial errors
    # write calibration code with gui
    # write gui for controlling joints
