import numpy as np
import tkinter as tk
import argparse

from nuro_arm.robot.robot_arm import RobotArm
from nuro_arm.constants import GRIPPER_CLOSED, GRIPPER_OPENED

class GUI(tk.Frame):
    def __init__(self, parent, robot):
        tk.Frame.__init__(self, parent)
        self.robot = robot
        self.controller = robot.controller
        self.is_active = True
        self.color_active = "#ff5703"
        self.color_active_hl = "#ff7733"
        self.color_passive = "#007826"
        self.color_passive_hl = "#00ad37"

        self.cycle_time = 50

        self.initialize()
        self.last_arm_command = None
        self.last_gripper_command = None

        self.update()

    def initialize(self):
        self.header = tk.Frame(self)
        self.body = tk.Frame(self)
        self.body.grid_columnconfigure(0, weight=1)
        self.footer = tk.Frame(self)
        self.footer.grid_columnconfigure(0, weight=1)
        self.footer.grid_rowconfigure(0, weight=1)

        self.btn_home = tk.Button(self.footer,fg="#036ffc",
                                     text="HOME", font=('bold'),
                                     command=self.go_home)
        self.btn_toggle_mode = tk.Button(self.footer,
                                         fg="#000000",
                                         bg=self.color_active,
                                         activebackground=self.color_active_hl,
                                         text="Active Mode", font=('bold'),
                                         command=self.toggle_mode)

        self.scales = []
        labels = []
        scale_width = 300

        # create scales for arm joints
        for i in range(len(self.controller.arm_joint_ids)):
            joint_name = self.robot.joint_names[i]
            joint_id = self.controller.arm_joint_ids[i]

            label = tk.Label(master=self.body, text=f"{joint_name}:")
            scale = tk.Scale(master=self.body,
                             from_=self.controller.arm_joint_limits[0,i],
                             to=self.controller.arm_joint_limits[1,i],
                             resolution=1e-3,
                             orient="horizontal",
                             length=scale_width,
                            )
            scale_value = self.controller.read_jpos([joint_id])[0]
            scale.set(scale_value)
            self.scales.append(scale)
            labels.append(label)

        # create scale for gripper
        label = tk.Label(master=self.body, text='gripper:')
        scale = tk.Scale(master=self.body,
                         from_=GRIPPER_CLOSED,
                         to=GRIPPER_OPENED,
                         resolution=0.01,
                         orient="horizontal",
                         length=scale_width,
                        )
        scale_value = self.robot.get_gripper_state()
        scale.set(scale_value)
        self.scales.append(scale)
        labels.append(label)

        # pack everything
        self.header.grid(row=0,column=0)
        self.body.grid(row=1,column=0)
        self.footer.grid(row=2,column=0, sticky="we")
        for i in range(len(labels)):
            # rows go in reverse so 'base' is at bottom
            row_id = len(labels)-i-1
            labels[i].grid(row=row_id, column=0, ipadx=6, ipady=2, sticky="se", pady=5)
            self.scales[i].grid(row=row_id, column=1, sticky="e", padx=(0,5), pady=5)

        self.btn_home.grid(row=0, column=0, sticky="w", padx=10, pady=10)
        self.btn_toggle_mode.grid(row=0, column=1, padx=10, pady=10)

    def go_home(self):
        self.robot.home()
        self.is_active = True
        [scl.set(jp) for scl, jp in zip(self.scales[:-1], self.controller.arm_jpos_home)]
        self.btn_toggle_mode.config(bg=self.color_active,
                                    activebackground=self.color_active_hl,
                                    text="Active Mode")

    def toggle_mode(self):
        self.is_active = not self.is_active
        if self.is_active:
            self.robot.active_mode()
            arm_jpos = self.robot.get_arm_jpos()
            [scl.set(jpos) for scl,jpos in zip(self.scales[:-1], arm_jpos)]
            self.scales[-1].set(self.robot.get_gripper_state())

            self.btn_toggle_mode.config(bg=self.color_active,
                                        activebackground=self.color_active_hl,
                                        text="Active Mode")
        else:
            self.last_arm_command = None
            self.last_gripper_command = None
            self.robot.passive_mode()
            self.btn_toggle_mode.config(bg=self.color_passive,
                                        activebackground=self.color_passive_hl,
                                        text="Passive Mode")

    def update(self):
        if self.is_active:
            # issue motion commands
            arm_jpos = [scl.get() for scl in self.scales[:-1]]
            if self.last_arm_command is None \
                        or not np.allclose(self.last_arm_command, arm_jpos):
                self.robot.move_arm_jpos(arm_jpos)
                self.last_arm_command = arm_jpos

            gripper_state = self.scales[-1].get()
            if self.last_gripper_command is None \
                        or self.last_gripper_command != gripper_state:
                self.robot.set_gripper_state(gripper_state)
                self.last_gripper_command = gripper_state
        else:
            # read positions and update scales
            gripper_state = self.robot.get_gripper_state()
            arm_jpos = self.robot.get_arm_jpos()
            self.scales[-1].set(gripper_state)
            [scl.set(jp) for scl, jp in zip(self.scales[:-1], arm_jpos)]

        self.after(self.cycle_time, self.update)


def move_with_gui(mode):
    '''Use GUI to control robot joints

    Currently, the gui issues direct commands without performing collision
    detection so the user should be careful about what is in the workplace
    '''
    robot = RobotArm(mode)

    root = tk.Tk()
    root.title('Simple Control of Robot Joints')
    gui = GUI(root, robot)
    gui.pack()
    root.mainloop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--sim', action="store_true")
    args = parser.parse_args()

    mode = 'sim' if args.sim else 'real'

    move_with_gui(mode)

