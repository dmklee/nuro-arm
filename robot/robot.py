import time
from robot.xarm import XArmRadians

class Robot():
    '''Higher level interface
    '''
    def __init__(self,
                 use_simulator = False,
                ):
        # load joint_limits
        if use_simulator:
            raise NotImplementedError
        else:
            self._xarm = XArmRadians()

        self.joint_limits = None
        self.joint_names = [s.name for s in self._xarm.servos]

        # constraints to prevent hitting camera stand/ground
        self.joint_constraints = None

    def calibrate(self):
        # check if they are sure
        # process may take a minute

        pass

    def move_joint(self,
                   joint_name : str,
                   angle : float,
                   duration : int,
                  ) -> bool:
        servo_id = self._get_servo_id(joint_name)
        exp_duration = self._xarm._move_servo_angle(servo_id, angle, duration)

        t_start = time.time()
        old_angle = self._xarm._read_servo_pos_angle(servo_id)
        while time.time() - t_start < 1.1*exp_duration:
            time.sleep(0.1)
            current_angle = self._xarm._read_servo_pos_angle(servo_id)

            # check if motion has stopped
            if abs(current_angle - old_angle) < self._xarm.BITS2RADIANS:
                if abs(current_angle - angle) < self._xarm.ANGLE_PRECISION:
                    return True
                # movement has failed
                break
            old_angle = current_angle

        # set servo to current angle so it does not keep trying to move
        self._xarm._move_servo_angle(servo_id,
                                     current_angle,
                                     duration=0)
        return False

    def read_joint(self, joint_name):
        servo_id = self._get_servo_id(joint_name)

        return self._xarm._read_servo_pos_angle(servo_id)

    def _get_servo_id(self, joint_name):
        assert joint_name in self.joint_names, 'invalid joint'
        servo_id = self._xarm.servos[joint_name]
        return servo_id

if __name__ == "__main__":
    robot = Robot()
    joint = 'gripper'
    print(robot.read_joint(joint))
    print(robot.move_joint(joint, -2.7, 500))
    print(robot.read_joint(joint))
    # while True:
        # robot._xarm.power_off()
        # print(robot._xarm._read_servo_pos(
            # robot._get_servo_id(joint)
        # ))
        # time.sleep(0.1)
