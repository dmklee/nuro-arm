import numpy as np
import tkinter as tk
from PIL import ImageTk, Image

class Colors:
    YES = '#51b442'
    NO = '#a03939'
    NEUTRAL = '#cacaca'
    BLACK = '#2e2e2e'
    ALARM = '#c30b0b'

class Popup:
    def __init__(self,
                 title='',
                 text='',
                 text_color=Colors.BLACK,
                 button_names=['OK','Cancel'],
                 button_colors=[Colors.YES, Colors.NO],
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

class ImagePopup(Popup):
    def __init__(self,
                 title='',
                 text='',
                 text_color=Colors.BLACK,
                 button_names=['OK','Cancel'],
                 button_colors=[Colors.YES, Colors.NO],
                 images=[],
                 image_shape=(200,200),
               ):
        super().__init__(title, text, text_color, button_names, button_colors)

        images_frame = tk.Frame(self.root)
        images_frame.grid(row=1, sticky='NESW', padx=6)
        H,W = image_shape

        for i, img_ in enumerate(images):
            if isinstance(img_, str):
                img = Image.open(img_)
            elif isinstance(img_, np.ndarray):
                # image coming from cv so it will be in BGR 
                img = Image.fromarray(img_[:,:,::-1])
            else:
                raise TypeError
            img = img.resize((W,H), Image.ANTIALIAS)
            img = ImageTk.PhotoImage(img)
            images_frame.grid_columnconfigure(i, weight=1)
            img_canvas = tk.Canvas(images_frame, width=W, height=H, bg='#ffffff')

            img_canvas.image = img
            img_canvas.grid(row=0, column=i, sticky='NS', padx=2)
            img_canvas.create_image(1,1, anchor='nw', image=img)

class VideoPopup(Popup):
    def __init__(self,
                 video_cap,
                 title='',
                 text='',
                 text_color=Colors.BLACK,
                 button_names=['OK','Cancel'],
                 button_colors=[Colors.YES, Colors.NO],
                 image_shape=(375, 500),
               ):
        super().__init__(title, text, text_color, button_names, button_colors)
        self.cap = video_cap
        self.image_shape = image_shape

        video_frame = tk.Frame(self.root)
        video_frame.grid(row=1, sticky='NESW', padx=6)
        H,W = image_shape

        # image coming from cv so it will be in BGR 
        img = Image.fromarray(self.cap.read()[:,:,::-1])
        img = img.resize((W,H), Image.ANTIALIAS)
        img = ImageTk.PhotoImage(img)
        video_frame.grid_columnconfigure(0, weight=1)
        self.img_canvas = tk.Canvas(video_frame, width=W, height=H, bg='#ffffff')

        self.img_canvas.image = img
        self.img_canvas.grid(row=0, column=0, sticky='NS', padx=2)
        self.img_on_canvas = self.img_canvas.create_image(1,1, anchor='nw', image=img)

        self.root.after(100, self.update_image)

    def update_image(self):
        H,W = self.image_shape
        img = Image.fromarray(self.cap.read()[:,:,::-1])
        img = img.resize((W,H), Image.ANTIALIAS)
        img = ImageTk.PhotoImage(img)
        self.img_canvas.itemconfigure(self.img_on_canvas, image=img)
        self.img_canvas.image = img
        self.root.after(100, self.update_image)
