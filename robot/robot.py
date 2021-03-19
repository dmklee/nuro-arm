from robot.xarm import XArmController

class AdvancedXArmController(XArmController):
    '''Higher level interface
    '''
    def _move_servo(self, servo_id, pos, duration=1000):

        duration = int(max(duration, pos_delta / self.MAX_SPEED))
        return super()._move_servo(servo_id, pos, duration)

    def move_smart(self, 

if __name__ == "__main__":
    arm = SafeXArmController()
    arm._move_servo(arm.servos.base, 400, 10)
    while True:
        pass
