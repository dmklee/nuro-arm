import time
from neu_ro_arm.robot.robot_arm import RobotArm

# define arm joint position sequences
go_to_block = [[-0.56, -0.03, 0.00, 0.00, 0.0],
               [-0.56, -0.85, 1.64, 0.72, 0.0],
               [-0.56, -1.21, 1.84, 1.05, 0.0],
               [-0.54, -0.01, 1.60, 1.07, 0.0],
               [-0.54,  0.14, 1.59, 1.07, 0.0]]

go_to_dropoff = [[-0.54, -1.20, 1.78, 1.07, 0.0],
                 [ 0.65, -1.20, 1.78, 1.07, 0.0],
                 [ 0.65, -0.15, 1.63, 1.07, 0.0]]

# connect to arm 
robot = RobotArm()

# start at home with gripper open
robot.home()
robot.open_gripper()

# follow sequence to go to cube
for arm_jpos in go_to_block:
    robot.move_arm_jpos(arm_jpos)
    time.sleep(0.5)

################################
# Feedback code should go here
################################

robot.close_gripper()

################################
# End of feedback code
################################

# robot should now have a cube
# follow sequence to drop off block
for arm_jpos in go_to_dropoff:
    robot.move_arm_jpos(arm_jpos)
    time.sleep(0.5)

# drop off block
robot.open_gripper()

# return home
robot.home()
