import tkinter as tk
from tkinter import ttk
import numpy as np
import time
from threading import Thread
import os

from nuro_arm.robot.robot_arm import RobotArm

class MovementDispatcher(Thread):
    def __init__(self, robot, table, idxs):
        '''Async thread to handle issuing commands to robot without hanging gui
        '''
        super().__init__()
        self.robot = robot
        self.table = table
        self.idxs = idxs
        self.running=True

    def run(self):
        '''Issue all commands sequentially.  Currently, the arm movement is
        performed first, followed by gripper movement
        '''
        children = self.table.get_children()
        for i in self.idxs:
            self.table.selection_set(children[i])
            values = self.table.item(children[i])['values']
            full_arm_state = [float(a) for a in values]
            arm_jpos = full_arm_state[:-1]
            gripper_state = full_arm_state[-1]

            robot.move_arm_jpos(arm_jpos)
            robot.set_gripper_state(gripper_state)
            if not self.running:
                break

class GUI(tk.Frame):
    def __init__(self, parent, robot):
        tk.Frame.__init__(self, parent)
        self.robot = robot
        self.data = []
        self.moving = False
        self.robot.passive_mode()

        self.initialize()

    def initialize(self):
        self.header = tk.Frame(self)
        self.header.grid_columnconfigure(0, weight=1)
        self.footer = tk.Frame(self)

        field_names = list(robot.joint_names)
        self.table = ttk.Treeview(self,
                             show='headings',
                             columns=list(range(len(field_names))),
                             height=12,
                             selectmode="browse")

        for i, name in enumerate(field_names):
            self.table.column(i, width=10*(len(name)+2),
                              anchor='center', stretch='no')
            self.table.heading(i, text=name)

        vscrollbar = ttk.Scrollbar(self,
                                   orient='vertical',
                                   command=self.table.yview)
        vscrollbar.grid(row=1,column=1, sticky="ns")
        self.table.configure(yscrollcommand=vscrollbar.set)

        self.btn_play = tk.Button(self.header, bg="#ffffff", fg="#069621",
                                  text="Play All", font=('Helvetica','12','bold'),
                                  command=lambda: self.toggle_move('all')
                                 )
        self.btn_go = tk.Button(self.header, bg="#ffffff", fg="#069621",
                                text="Move to", font=('Helvetica','12','bold'),
                                command=lambda: self.toggle_move('single')
                               )
        self.btn_save = tk.Button(self.header, bg="#ffffff", fg="#bd5e00",
                                  text="Save", font=('Helvetica','12','bold'),
                                  command=self.save)
        self.btn_load = tk.Button(self.header, bg="#ffffff", fg="#bd5e00",
                                  text="Load", font=('Helvetica','12','bold'),
                                  command=self.load)
        self.btn_append = tk.Button(self.footer, bg="#ffffff", fg="#0740c9",
                                 text="Append", font=('Helvetica','12','bold'),
                                 command=self.append,)
        self.btn_insert = tk.Button(self.footer, bg="#ffffff", fg="#0740c9",
                                 text="Insert", font=('Helvetica','12','bold'),
                                 command=self.insert,)
        self.btn_move_up = tk.Button(self.footer, bg="#ffffff", fg="#000000",
                                     text="Shift Up", font=('Helvetica','12','bold'),
                                     command=self.move_up,)
        self.btn_move_down = tk.Button(self.footer, bg="#ffffff", fg="#000000",
                                       text="Shift Down", font=('Helvetica','12','bold'),
                                       command=self.move_down,)
        self.btn_delete = tk.Button(self.footer, bg="#ffffff", fg="#ff0000",
                                    text="Delete", font=('Helvetica','12','bold'),
                                    command=self.delete,)

        # pack 
        self.header.grid(row=0, pady=4, sticky='we')
        self.table.grid(row=1, sticky='we')
        self.footer.grid(row=2, sticky='e', pady=4)

        self.btn_play.grid(row=0, column=0, padx=10, sticky='w')
        self.btn_go.grid(row=0, column=0, padx=10)
        self.btn_save.grid(row=0, column=1, padx=10, sticky='e')
        self.btn_load.grid(row=0, column=2, padx=10, sticky='e')

        self.btn_move_up.grid(row=0, column=0,padx=4)
        self.btn_move_down.grid(row=0, column=1,padx=4)
        self.btn_append.grid(row=0, column=2,padx=4)
        self.btn_insert.grid(row=0, column=3,padx=4)
        self.btn_delete.grid(row=0, column=4,padx=4)

    def append(self):
        if not self.moving:
            new = self.robot.get_arm_jpos()
            new.append(self.robot.get_gripper_state())
            self.data.append(new)
            id_ = self.table.insert('','end', values=[f'{a:.2f}' for a in new])

            # select newest by default
            self.table.selection_set(id_)

    def insert(self):
        if not self.moving:
            selected = self.table.selection()
            if len(selected) == 0:
                return self.append()
            selected = selected[0]
            selected_id = self.table.index(selected)

            new = self.robot.get_arm_jpos()
            new.append(self.robot.get_gripper_state())
            self.data.append(new)

            id_ = self.table.insert('',selected_id, values=[f'{a:.2f}' for a in new])

            # select newest by default
            self.table.selection_set(id_)

    def move_up(self):
        if not self.moving:
            selected = self.table.selection()
            if len(selected) == 0:
                return
            selected = selected[0]
            selected_id = self.table.index(selected)
            if selected_id > 0:
                self.table.move(selected, '', selected_id-1)

    def move_down(self):
        if not self.moving:
            selected = self.table.selection()
            if len(selected) == 0:
                return
            selected = selected[0]

            selected_id = self.table.index(selected)
            self.table.move(selected, '', selected_id+1)

    def delete(self):
        if not self.moving:
            selected = self.table.selection()
            if len(selected) == 0:
                return
            selected = selected[0]

            to_select = self.table.prev(selected)
            if to_select == '':
                # we need to find something to select if there are still rows
                to_select = self.table.next(selected)

            self.table.delete(selected)
            self.table.selection_set(to_select)

    def toggle_move(self, mode):
        assert mode in ('single', 'all')
        if self.moving:
            self.moving = False
            self.change_buttons_state('normal')
        else:
            if mode == 'single':
                try:
                    idxs = [self.table.index(self.table.selection()[0])]
                except IndexError:
                    idxs = []
            else:
                idxs = range(len(self.table.get_children()))

            if len(idxs) == 0:
                return

            self.change_buttons_state('disabled')
            self.btn_play.config(text="STOP", fg="#ff0000")
            dispatcher = MovementDispatcher(self.robot,
                                            self.table,
                                            idxs)
            self.moving = True
            dispatcher.start()
            self.monitor(dispatcher)

    def monitor(self, thread):
        if thread.is_alive() and self.moving:
            self.after(100, lambda : self.monitor(thread))
        else:
            self.change_buttons_state('normal')
            self.btn_play.config(text="Play All", fg="#069621")
            thread.running = False
            self.moving = False
            self.robot.passive_mode()

    def change_buttons_state(self, state):
        assert state in ('normal', 'disabled')
        self.btn_go['state'] = state
        self.btn_save['state'] = state
        self.btn_load['state'] = state
        self.btn_move_up['state'] = state
        self.btn_move_down['state'] = state
        self.btn_append['state'] = state
        self.btn_insert['state'] = state
        self.btn_delete['state'] = state

    def save(self):
        if len(self.table.get_children()) == 0:
            print('You must add joint positions before saving.')
            return

        def check_file():
            filename = entry.get()
            if filename == '':
                print('Filename cannot be empty')
            else:
                filepath = os.path.dirname(os.path.abspath(__file__))
                filename = filename + '.npy'
                data = []
                for child in self.table.get_children():
                    data.append(self.table.item(child)['values'])
                data = np.array(data).astype(float)
                np.save(os.path.join(filepath, filename), data)
                popup.destroy()

        popup = tk.Toplevel()
        popup.winfo_toplevel().title('')
        popup.grid_columnconfigure(0, weight=1)
        popup.grid_rowconfigure(0, weight=1)

        label = tk.Label(popup, text="File name: ")
        entry = tk.Entry(popup)
        button = tk.Button(popup, text='save', command=check_file)
        label.grid(row=0, column=0, padx=3, pady=3)
        entry.grid(row=0, column=1)
        button.grid(row=0, column=2)

    def load(self):
        def load_file():
            selected = table.selection()
            if len(selected) == 0:
                return
            filename = table.item(selected[0])['values'][0]
            self.table.delete(*self.table.get_children())
            for data in np.load(os.path.join(filepath,filename)):
                item = self.table.insert('','end', values=[f'{d:.2f}' for d in data])
            self.table.selection_set(item)
            popup.destroy()

        popup = tk.Toplevel()
        popup.winfo_toplevel().title('')
        popup.grid_columnconfigure(0, weight=1)
        popup.grid_rowconfigure(0, weight=1)

        # check if files exist
        filepath = os.path.dirname(os.path.abspath(__file__))
        files = [f for f in next(os.walk(filepath))[2] if f.endswith('.npy')]
        if len(files) == 0:
            tk.Label(popup,
                     text="No files were found.",
                     fg='#990000',
                     font=('Helvetica','12','bold'),
                    ).grid(padx=10, pady=10)
            self.after(800, popup.destroy)
            return

        table = ttk.Treeview(popup,
                             show='headings',
                             columns=(0,),
                             height=8,
                             selectmode="browse")
        table.column(0, anchor='center')
        table.heading(0, text='Select a File')
        for f in files:
            table.insert('','end', values=[f,])
        table.selection_set(table.get_children()[0])

        vscrollbar = ttk.Scrollbar(popup,
                                   orient='vertical',
                                   command=table.yview)
        button = tk.Button(popup, text='load', font=('bold'),
                           command=load_file)

        table.grid(row=1,column=0, sticky="ns", pady=5, padx=(3,0))
        vscrollbar.grid(row=1,column=1, sticky="ns", pady=5)
        button.grid(row=1, column=2, sticky='n',padx=10, pady=5)

if __name__ == "__main__":
    robot = RobotArm()
    root=tk.Tk()
    root.title('Record and Playback Arm Configurations')
    gui = GUI(root, robot)
    gui.pack()
    root.mainloop()
