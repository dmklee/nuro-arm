import numpy as np
import serial
import time

from joint_state import JointStates

class Braccio:
    #TODO: handle erros over serial port
    def __init__(self, port_name='/dev/ttyACM0'):
        self.move_speed = 100
        self.port = None
        self.connect_serial(port_name)

        self.joint_states = JointStates()

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

    def go(self, joint_states):
        command_string = ('P{:d},{:d},{:d},{:d},{:d},{:d},{:d}\n').format(
            *joint_states.to_degrees(round=True), self.move_speed)
        self._write_serial(command_string)

    def go_home(self):
        self._write_serial('H\n')

    def power_on(self):
        self._write_serial('1\n')

    def power_off(self):
        if self.port is not None:
            self._write_serial('0\n')

    def __del__(self):
        self.power_off()

    def _write_serial(self, string):
        self.port.write(string.encode())
        print(self.port.readline().decode())

if __name__ == "__main__":
    robot = Braccio()
    print(robot.joint_states.get_gripper_position())
    for _ in range(1):
        robot.open_gripper()
        print(robot.joint_states.get_gripper_position())
        time.sleep(1)
        robot.close_gripper()
        time.sleep(1)

