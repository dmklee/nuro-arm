
from nuro_arm.robot.robot_arm import RobotArm

robot_positions = [(0, -0.3, 0.012),
                   (0, 0, 0.012),
                   (0, 0.3, 0.012)]

robots = []
pb_client = None
for pos in robot_positions:
    robot = RobotArm('sim',
                     headless=False,
                     pb_client=pb_client)
    robot._sim.reset_robot_base(pos)
    pb_client = robot.get_pb_client()
    robots.append(robot)

while 1:
    # robot.controller.timestep()
    pass


