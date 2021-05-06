import tkinter as tk
import numpy as np
import time
from threading import Thread

from neu_ro_arm.robot.robot_arm import RobotArm

class MovementDispatcher(Thread):
    def __init__(self, robot, data, wp_frame):
        super().__init__()
        self.robot = robot
        self.data = data
        self.wp_frame = wp_frame
        self.running=True

    def run(self):
        for i, full_arm_state in enumerate(self.data):
            arm_jpos = full_arm_state[:-1]
            gripper_state = full_arm_state[-1]
            self.wp_frame.highlight_row(i)
            robot.move_arm_jpos(arm_jpos)
            robot.set_gripper_state(gripper_state)
            self.wp_frame.unhighlight_row(i)
            if not self.running:
                break

class WaypointsFrame(tk.Frame):
    def __init__(self, parent, field_names):
        tk.Frame.__init__(self, parent)

        self.field_names = field_names
        self.colors = {'highlight' : "#56a95b",
                       'normal' : "#ffffff"}
        self.rows = []

        #TODO: get the canvas to expand as necessary without specifying width
        canvas_width = 510
        self.canvas = tk.Canvas(self, width=canvas_width,
                                borderwidth=0, background="#ffffff")
        self.frame = tk.Frame(self.canvas, background="#ffffff")
        self.frame.grid_columnconfigure(0,weight=1)

        self.vsb = tk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=self.vsb.set)

        self.vsb.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)
        self.canvas_frame = self.canvas.create_window((0,0),
                                                      window=self.frame,
                                                      anchor="nw",
                                                      tags="self.frame")

        self.frame.bind("<Configure>", self.onFrameConfigure)

        self.initialize()

    def add(self, values):
        row_id = len(self.rows) + 1
        new_row = []

        lbl = tk.Label(self.frame, width=3, borderwidth=1,
                       fg="#494949", relief="solid", text=str(row_id))
        lbl.grid(row=row_id, column=0, sticky='nswe')
        new_row.append(lbl)

        for i,v in enumerate(values):
            text = v if isinstance(v, str) else f"{v:0.2f}"
            lbl = tk.Label(self.frame, borderwidth=1,
                           text=text, relief="solid", bg=self.colors['normal'])
            lbl.grid(row=row_id, column=i+1, sticky='nswe')
            new_row.append(lbl)

        self.rows.append(new_row)

    def clear(self):
        for row in self.rows:
            for lbl in row:
                lbl.destroy()
        self.rows = []

    def initialize(self):
        tk.Label(self.frame,
                 borderwidth=1,
                 text="  ",
                 relief="solid",
                 font=('bold'),
                ).grid(row=0, column=0, ipadx=8, sticky='nwse')

        for i,v in enumerate(self.field_names):
            tk.Label(self.frame,
                     borderwidth=1,
                     text=v,
                     relief="solid",
                     font=('bold'),
                    ).grid(row=0, column=i+1, ipadx=8)

    def highlight_row(self, row_id):
        [lbl.config(bg=self.colors['highlight']) for lbl in self.rows[row_id][1:]]

    def unhighlight_row(self, row_id):
        [lbl.config(bg=self.colors['normal']) for lbl in self.rows[row_id][1:]]

    def onFrameConfigure(self, event):
        '''Reset the scroll region to encompass the inner frame'''
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

class GUI(tk.Frame):
    def __init__(self, parent, robot):
        tk.Frame.__init__(self, parent)
        self.robot = robot
        self.data = []
        self.moving = False
        self.robot.passive_mode()

        self.header = tk.Frame(self)
        self.body = tk.Frame(self)
        self.footer = tk.Frame(self)

        field_names = list(robot.joint_names) + ['gripper']
        self.waypoints_frame = WaypointsFrame(self.body, field_names)

        self.play_button = tk.Button(self.header, bg="#ffffff", fg="#069621",
                                     text="Play", font=('bold'),
                                     command=self.toggle_arm_motion)
        self.clear_button = tk.Button(self.header, bg="#ffffff", fg="#d61111",
                                      text="Clear", font=('bold'),
                                      command=self.clear)
        self.add_button = tk.Button(self.footer, bg="#ffffff", fg="#0740c9",
                                    text="Add", font=('bold'),
                                    command=self.add)

        # pack 
        self.header.pack(side="top")
        self.body.pack(side="top")
        self.footer.pack(side="top")
        self.waypoints_frame.pack(side="top", fill="both")
        self.clear_button.pack(side="right", fill="x", padx=20)
        self.play_button.pack(side="right", fill="x", padx=20)
        self.add_button.pack(side="right", fill="x")

    def add(self):
        # get data 
        if self.moving:
            print('Cannot add while arm is in motion')
        else:
            new = self.robot.get_arm_jpos()
            new.append(self.robot.get_gripper_state())
            self.data.append(new)
            self.waypoints_frame.add(new)

    def clear(self):
        if self.moving:
            print('Cannot clear while arm is in motion')
        else:
            self.data = []
            self.waypoints_frame.clear()

    def toggle_arm_motion(self):
        if self.moving:
            self.moving = False
        else:
            self.play_button.config(text="STOP", fg="#ff0000")
            dispatcher = MovementDispatcher(self.robot, self.data.copy(),
                                            self.waypoints_frame)
            self.moving = True
            dispatcher.start()
            self.monitor(dispatcher)

    def monitor(self, thread):
        if thread.is_alive() and self.moving:
            self.after(100, lambda : self.monitor(thread))
        else:
            self.play_button.config(text="Play", fg="#069621")
            thread.running = False
            self.moving = False
            self.robot.passive_mode()

if __name__ == "__main__":
    robot = RobotArm()
    root=tk.Tk()
    root.title('Record and Playback Arm Configurations')
    gui = GUI(root, robot)
    gui.pack()
    root.mainloop()
