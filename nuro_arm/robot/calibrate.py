import numpy as np

from nuro_arm.tk_utils import ImagePopup, Popup, Colors
from nuro_arm.robot.xarm_controller import XArmController
from nuro_arm.constants import XARM_CONFIG_FILE, IMAGES_DIR

def calibrate_xarm():
    '''Calibrates servos in xArm so that the internal motion planner is accurate

    Calibration info is saved to a config file so this only needs to be performed
    once after the robot is assembled. For more information, see the
    installation guide
    '''
    # INTRO
    xarm = XArmController()
    xarm.power_off_servos()
    popup = Popup(
        title='xArm Calibration: step 0 of 4',
        text='The calibration process will take a minute or two. Please ensure \n' \
             'that the area around the arm is free of objects and people. \n\n' \
             'Press BEGIN to start the calibration process.',
        button_names=['BEGIN', 'QUIT']
    )
    if popup.response() != 'BEGIN':
        exit()

    # SERVO OFFSETS
    popup = ImagePopup(
        title='xArm Calibration: step 1 of 4',
        text='The first step is to set the home configuration of the arm. \n' \
             'Please move the arm such that it is vertically aligned as shown in \n' \
             'the images below. You do not need to adjust the gripper fingers yet, \n' \
             'although the hand itself should be properly aligned as shown. \n\n' \
             'Click CONTINUE once the arm is in the home configuration.',
        images=[f'{IMAGES_DIR}/home.png'],
        image_shape=(250,250),
        button_names=['CONTINUE', 'QUIT']
    )
    if popup.response() != 'CONTINUE':
        exit()
    ret, arm_servo_offsets = xarm._reset_servo_offsets()
    if ret == False:
        joint_ids = [j_id for j_id,offset in arm_servo_offsets.items() if abs(offset) > 127]
        joint_names = [xarm.get_joint_name(j_id) for j_id in joint_ids]
        popup = Popup(
            title='Calibration Warning',
            text='We have detected an error in the following servos: \n' \
                 f'{joint_names}\n\n' \
                 'You will need to reinstall these servos. Refer to installation \n' \
                 'guide for instructions on reinstalling servos.',
            text_color=Colors.ALARM,
            button_names=['CLOSE'],
            button_colors=[Colors.NEUTRAL]
        )()
        exit()

    # SERVO DIRECTIONS
    xarm.power_off_servos()
    while 1:
        popup = ImagePopup(
            title='xArm Calibration: step 2 of 4',
            text='Next, we need to put the arm in a bent configuration, to evaluate what\n'\
                 'direction the servos are oriented. Please move the arm such that it looks\n'\
                 'like the pictures below. You do not need to be exact, but ensure that the\n'\
                 'arm is bent. You do not need to adjust the gripper fingers.\n\n'\
                 'Click CONTINUE once the arm is in the bent configuration.',
            images=[f'{IMAGES_DIR}/bent_arm.png'],
            image_shape=(200,400),
            button_names=['CONTINUE', 'QUIT']
        )
        if popup.response() != 'CONTINUE':
            exit()
        arm_jpos = xarm.read_arm_jpos()

        arm_joint_directions = {}
        for i, j_id in enumerate(xarm.arm_joint_ids):
            # base and wristRotation joints are always same direction
            if j_id in [6,2]:
                arm_joint_directions[j_id] = 1
            else:
                arm_joint_directions[j_id] = np.sign(arm_jpos[i]) \
                                             * xarm.arm_joint_directions[j_id]

        if any([v==0 for v in arm_joint_directions.values()]):
            joint_names = [xarm.get_joint_name(j_id)
                           for j_id, v in arm_joint_directions.items() if v==0]
            popup = Popup(
                title='Calibration Warning',
                text='We have detected an error in the bent configuration, it \n' \
                     'appears that you did not bend the following joint(s):\n' \
                     f'{joint_names}\n\n' \
                     'Please close this window and try again. ',
                text_color=Colors.ALARM,
                button_names=['CLOSE'],
                button_colors=[Colors.NEUTRAL]
            )()
        else:
            break

    # GRIPPER
    while 1:
        popup = ImagePopup(
            title='xArm Calibration: step 3 of 4',
            text='Finally, we need to set the limits of the gripper fingers. \n' \
                 'Using two hands, gently close the gripper fingers until they\n'\
                 'are touching like shown in the picture.\n\n'\
                 'Click CONTINUE once the gripper has been closed.',
            images=[f'{IMAGES_DIR}/gripper_closed.png'],
            image_shape=(250,250),
            button_names=['CONTINUE', 'QUIT']
        )
        if popup.response() != 'CONTINUE':
            exit()
        gripper_closed = xarm.read_jpos(xarm.gripper_joint_ids)[0]

        popup = ImagePopup(
            title='xArm Calibration: step 4 of 4',
            text='Now, again using two hands, open the gripper fingers (pushing\n' \
                 'down works better than pulling apart).  The lower parts of the\n'\
                 'finger should be perpindicular to the hand as shown in the picture.\n\n'\
                 'Click CONTINUE once the gripper has been opened.',
            images=[f'{IMAGES_DIR}/gripper_opened.png'],
            image_shape=(250,250),
            button_names=['CONTINUE', 'QUIT']
        )
        if popup.response() != 'CONTINUE':
            exit()
        gripper_opened = xarm.read_jpos(xarm.gripper_joint_ids)[0]

        # check if gripper values seem reasonable
        if (gripper_closed - gripper_opened) < 1.5:
            popup = Popup(
                title='Calibration Warning',
                text='We have detected an error in the gripper calibration.\n' \
                     'Please close this window and try again.',
                text_color=Colors.ALARM,
                button_names=['CLOSE'],
                button_colors=[Colors.NEUTRAL]
            )()
        else:
            break

    # update config file
    try:
        data = np.load(XARM_CONFIG_FILE, allow_pickle=True).item()
    except FileNotFoundError:
        data = dict()
    data[xarm.serial_number] = {
        'arm_joint_directions' : arm_joint_directions,
        'gripper_joint_limits' : np.array(((gripper_closed,), (gripper_opened,))),
        'servo_offsets' : arm_servo_offsets,
    }
    np.save(XARM_CONFIG_FILE, data)

    popup = Popup(
        title='Success',
        text='xArm was calibrated successfully. All data has been logged\n' \
             'and it is now safe to use the arm.',
        button_names=['CLOSE'],
        button_colors=[Colors.NEUTRAL]
    )()

if __name__ == "__main__":
    calibrate_xarm()

