Example Scripts
===============

Hardcoded Arm Movements
-----------------------
create a sequence of arm joint positions and use for loop to go over them

.. code-block:: python

    from nuro_arm import RobotArm

    robot = RobotArm()
    jpos = [0, 0, 0, 0, 0]

    for i in range(10):
        jpos[1] += i/10
        robot.move_arm_jpos(jpos)


Using Feedback from Joint Positions
-----------------------------------

Here is an example where we use the state of the gripper to determine whether an object was grasped.  The program keeps attempting to close the gripper until it detects an object in the gripper (i.e. gripper could not be fully closed), at which point it drops the object off at another location.

.. code-block:: python

    from nuro_arm import RobotArm

    robot = RobotArm()
    grasp_jpos = [-0.2, 0, 0.5, 0, 0]
    drop_jpos = [0.2, 0, 0.5, 0, 0]

    robot.open_gripper()
    robot.move_arm_jpos(grasp_jpos)

    while True:
        robot.close_gripper()
        gripper_state = robot.get_gripper_state()

        # if something in gripper, drop it off
        if gripper_state > 0.1:
            robot.move_arm_jpos(drop_jpos)
            robot.open_gripper()
            break

        robot.open_gripper()


Another example of arm early stoppage to determine object location:

Top-Down Grasping
-----------------
show IK solution example

Nudging a Cube
--------------
show cube detection and movement
