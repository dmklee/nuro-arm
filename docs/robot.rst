Using Robot Arm
===============

Connecting to Robot
-------------------

.. code-block:: python
    
    from nuro_arm import RobotArm

    # connect to robot
    controller_type = 'real' # or 'sim'
    robot = RobotArm(controller_type)

    # turn off motors, useful for guiding robot by hand
    robot.passive_mode()

    # turn on motors
    robot.active_mode()

Joint Angle Control
-------------------

.. code-block:: python
    
    # joint angles in radians, from base to wrist_rotation
    jpos = [0.3, 0, 0, 0, 0]

    # sends command, returns once motion stops
    robot.move_arm_jpos(jpos)

    # see achieved joint angles
    achieved_jpos = robot.get_arm_jpos()

End Effector Control
--------------------

Add picture of coordinate frames of robot base and end effector

.. code-block:: python
    
    # end effector position, units in meters
    ee_pos = [0.2, 0.0, 0.1]

    # sends command, returns once motion stops
    robot.move_hand_to(ee_pos)

    # see achieved end effector position
    achieved_ee_pos = robot.get_hand_pose()

Using Gripper
-------------

.. code-block:: python
    
    # opens gripper, returns once motion stops
    robot.open_gripper()

    # closes_gripper, returns once motion stops
    robot.open_gripper()

    # for more fine-grained gripper control, use float ranging from 
    # 0 (fully closed) to 1 (fully opened)
    robot.set_gripper_state(0.1)

    # get current gripper state (ranging from 0 to 1)
    gripper_state = robot.get_gripper_state()

Collision Detection
-------------------
Coming soon... 

Advanced Motions
----------------
Coming soon... 

Calibration and Maintenance
---------------------------

