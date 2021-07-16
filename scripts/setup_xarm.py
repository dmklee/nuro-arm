import numpy as np
import tkinter as tk
from PIL import ImageTk, Image

from neu_ro_arm.robot.xarm_controller import XArmController

YES = '#51b442'
NO = '#a03939'
NEUTRAL = '#cacaca'
BLACK = '#2e2e2e'
ALARM = '#c30b0b'
class Popup:
    def __init__(self,
                 title='',
                 text='',
                 text_color=BLACK,
                 images=[],
                 button_names=['OK','Cancel'],
                 button_colors=[YES, NO],
                 image_shape=(200,200),
                ):

        def press_gen(name):
            def press():
                self.ret = name
                self.root.destroy()
            return press

        self.ret = None
        self.root = tk.Tk()
        self.root.winfo_toplevel().title(title)
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(0, weight=1)

        # text
        text_frame = tk.Frame(self.root, pady=4, padx=6)
        text_frame.grid_columnconfigure(0, weight=1)
        text_frame.grid(row=0, sticky='NSEW')
        text_label = tk.Label(text_frame, text=text, fg=text_color,
                               font=('Helvetica','12','normal'))
        text_label.grid(sticky='NSEW')

        # images
        images_frame = tk.Frame(self.root)
        images_frame.grid(row=1, sticky='NESW', padx=6)
        H,W = image_shape

        for i, img_name in enumerate(images):
            images_frame.grid_columnconfigure(i, weight=1)
            img = Image.open(img_name)
            img = img.resize((W,H), Image.ANTIALIAS)
            img = ImageTk.PhotoImage(img)
            img_canvas = tk.Canvas(images_frame, width=W, height=H, bg='#ffffff')

            img_canvas.image = img
            img_canvas.grid(row=0, column=i, sticky='NS', padx=2)
            img_canvas.create_image(1,1, anchor='nw', image=img)

        # buttons
        buttons_frame = tk.Frame(self.root)
        buttons_frame.grid(row=2, sticky='NSEW', pady=8, padx=8)
        for i, name in enumerate(button_names):
            buttons_frame.grid_columnconfigure(i, weight=1)
            button = tk.Button(buttons_frame, text=name, bg=button_colors[i],
                               font=('Helvetica','12','bold'),
                               width=8, command=press_gen(name))
            button.grid(row=0, column=i, sticky='NS', padx=10)

    def response(self):
        return self()

    def __call__(self):
        self.root.mainloop()
        return self.ret

def calibrate(xarm):
    '''Calibrates servos in xArm so that the internal motion planner is accurate

    Calibration info is saved to a config file so this only needs to be performed
    once after the robot is assembled. For more information, see the
    installation guide
    '''
    # INTRO
    xarm.power_off_servos()
    popup = Popup(
        title='xArm Calibration: step 0 of 4',
        text='The calibration process will take a minute or two. Please ensure \n' \
             'that the area around the arm is free of objects and people. \n' \
             'Press BEGIN to start the calibration process.',
        button_names=['BEGIN', 'QUIT']
    )
    if popup.response() != 'BEGIN':
        exit()

    # SERVO OFFSETS
    popup = Popup(
        title='xArm Calibration: step 1 of 4',
        text='The first step is to set the home configuration of the arm. \n' \
             'Please move the arm such that it is vertically aligned as shown in \n' \
             'the images below. You do not need to adjust the gripper fingers yet, \n' \
             'although the hand itself should be properly aligned as shown. \n' \
             'Click CONTINUE once the arm is in the home configuration.',
        images=['images/xarm_calibration/home.png'],
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
            text_color=ALARM,
            button_names=['CLOSE'],
            button_colors=[NEUTRAL]
        )()
        exit()

    # SERVO DIRECTIONS
    xarm.power_off_servos()
    while 1:
        popup = Popup(
            title='xArm Calibration: step 2 of 4',
            text='Next, we need to put the arm in a bent configuration, to evaluate what\n'\
                 'direction the servos are oriented. Please move the arm such that it looks\n'\
                 'like the pictures below. You do not need to be exact, but ensure that the\n'\
                 'arm is bent. You do not need to adjust the gripper fingers.\n'\
                 'Click CONTINUE once the arm is in the bent configuration.',
            images=['images/xarm_calibration/bent_arm.png'],
            image_shape=(200,400),
            button_names=['CONTINUE', 'QUIT']
        )
        if popup.response() != 'CONTINUE':
            exit()
        arm_jpos = xarm.read_arm_jpos()
        arm_joint_directions = {i:s for i,s in zip(xarm.arm_joint_ids,np.sign(arm_jpos))}

        # base&wristRotation joints will always be in the same direction
        arm_joint_directions[xarm.get_joint_id('base')] = 1.
        arm_joint_directions[xarm.get_joint_id('wristRotation')] = 1.

        if any([v==0 for v in arm_joint_directions.values()]):
            popup = Popup(
                title='Calibration Warning',
                text='We have detected an error in the bent configuration.\n' \
                     'Please close this window and try again. ',
                text_color=ALARM,
                button_names=['CLOSE'],
                button_colors=[NEUTRAL]
            )()
        else:
            break

    # GRIPPER
    while 1:
        popup = Popup(
            title='xArm Calibration: step 3 of 4',
            text='Finally, we need to set the limits of the gripper fingers. \n' \
                 'Using two hands, gently close the gripper fingers until they\n'\
                 'are touching like shown in the picture.\n'\
                 'Click CONTINUE once the gripper has been closed.',
            images=['images/xarm_calibration/gripper_closed.png'],
            image_shape=(250,250),
            button_names=['CONTINUE', 'QUIT']
        )
        if popup.response() != 'CONTINUE':
            exit()
        gripper_closed = xarm._read_jpos(xarm.gripper_joint_ids)[0]

        popup = Popup(
            title='xArm Calibration: step 4 of 4',
            text='Now, again using two hands, open the gripper fingers (pushing\n' \
                 'down works better than pulling apart).  The lower parts of the\n'\
                 'finger should be perpindicular to the hand as shown in the picture.\n'\
                 'Click CONTINUE once the gripper has been opened.',
            images=['images/xarm_calibration/gripper_opened.png'],
            image_shape=(250,250),
            button_names=['CONTINUE', 'QUIT']
        )
        if popup.response() != 'CONTINUE':
            exit()
        gripper_opened = xarm._read_jpos(xarm.gripper_joint_ids)[0]

        # check if gripper values seem reasonable
        if (gripper_closed - gripper_opened) < 1.5:
            popup = Popup(
                title='Calibration Warning',
                text='We have detected an error in the gripper calibration.\n' \
                     'Please close this window and try again.',
                text_color=ALARM,
                button_names=['CLOSE'],
                button_colors=[NEUTRAL]
            )()
        else:
            break

    # save config file
    data = {
        'arm_joint_directions' : arm_joint_directions,
        'gripper_joint_limits' : np.array(((gripper_closed,),(gripper_opened,))),
        'servo_offsets' : arm_servo_offsets,
    }
    np.save(xarm.CONFIG_FILE, data)

    popup = Popup(
        title='Success',
        text='xArm was calibrated successfully. All data has been logged\n' \
             'and it is now safe to use the arm.',
        button_names=['CLOSE'],
        button_colors=[NEUTRAL]
    )()

if __name__ == "__main__":
    xarm = XArmController()
    calibrate(xarm)

