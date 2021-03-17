from robot.xarm import XArmController

class SafeXArmController(XArmController):
    '''Adds safety measures like maximum speed, joint_space constraints.
    Provides more high level motion commands
    '''
    MAX_SPEED = 0.5 # pos units per millisecond
    def _move_servo(self, servo_id, pos, duration=1000):
        current_pos = self._read_servo_pos(servo_id)
        pos_delta = abs(pos - current_pos)

        duration = int(max(duration, pos_delta / self.MAX_SPEED))
        return super()._move_servo(servo_id, pos, duration)

if __name__ == "__main__":
    arm = SafeXArmController()
    arm._move_servo(arm.servos.base, 400, 10)
    while True:
        pass
