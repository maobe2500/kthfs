#!/usr/bin/env python3
from tkinter import Tk, Frame, Menu, Label, Button, Entry, TOP, BOTH, BOTTOM
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

class Plotter:
    """
    A General Plotter that plots an arbitrary function

    Attributes:
        func: The fucntion to plot
        x: List containing the x values
        y: List containing the y values
        figure: Matplotlib figure object
        axes: Axes for the plot
        line: the plotted line used in animation

    Methods:
        init_animation: A neccecary function for the FuncAnimation to work
        init_plot: Sets up the x and y lists
        get_animation: runs the animation
        animate: A neccecary function for FuncAnimation
        plot: Plots the values in x and y lists
    """

    def __init__(self, func):
        """
        Parameters:
             func: A function to plot
        """
        self.func = func
        self.x = []
        self.y = []
        self.figure = plt.figure(figsize = (8,6), dpi=100)
        self.axes = self.figure.add_subplot()
        self.line, = self.axes.plot(self.x, self.y, 'r', linewidth=0.5)

        # Config
        plt.style.use('fivethirtyeight')
        plt.grid()

    def init_animation(self):
        """initializes plot"""
        self.line.set_data(self.x, self.y)
        return self.line,

    def init_plot(self, xlimit):
        """initializes value lists"""
        self.x = list(range(xlimit))
        self.y = [self.func(x) for x in self.x]

    def animate(self,i):
        """Runs every frame of animation"""
        self.x = list(range(i))
        self.y = [self.func(x) for x in range(i)]
        self.line.set_data(self.x, self.y)
        return self.line,


    def get_animation(self):
        """
        Runs animation and returns the variabal.  
        IMPORTANT:
        The animation variable must be saved otherwise the 
        animation gets garbage collected
        """
        animation = FuncAnimation(self.figure, self.animate, 
                                  init_func=self.init_animation, interval=20,
                                  frames=1000, blit=True)
        return animation

    def plot(self, xlimit):
        """Sets up the data and plots it"""
        self.init_plot(xlimit)
        self.axes.plot(self.x, self.y)
        plt.tight_layout()
        plt.show()


class GUI:
    """
    A GUI class that can handle menus, different pages
    and displaying graphs

    Attributes:
         root: Description
         main_frame: Description
         frames: Description
         entry_boxes: Description

    Methods:
         add_menu_item:
         add_frame:
         add_button:
         add_entry_box:

    """

    def __init__(self):
        self.root = Tk()
        self.main_frame = Frame(self.root)
        self.title = 'GUI Test'
        self.entry_boxes = {}

        # GUI Objects
        self.menu_area = Menu(self.root)

        # Config
        self.root.title(self.title)
        self.root.geometry('800x800')
        self.root.config(menu=self.menu_area)
        self.main_frame.pack(fill=BOTH, expand=1, padx=10, pady=10)

    def add_menu_item(self, label, command, last_item=False):
        """blah"""
        self.menu_area.add_command(label=label, command=command)
        if last_item:
            self.menu_area.add_command(label='exit', command=self.root.destroy)

    def add_frame(self, bg):
        """blah"""
        frame = Frame(self.root, bg=bg)
        frame.pack(padx=10, pady=10)
        return frame

    def add_button(self, label, command, frame, bind_key=None):
        """blah"""
        button = Button(frame, text=label, command=command)
        button.pack(side=TOP, padx=10, pady=10, fill=BOTH)
        self.root.bind(bind_key, command)

    def add_entry_box(self, label, frame):
        """blah"""
        self.entry_boxes[label] = Entry(frame, width = 4)
        Label(frame, text=label).pack(side=TOP, padx=10, pady=10)
        self.entry_boxes[label].pack(side=TOP, padx=20, pady=10)

    def main(self):
        """blah"""
        self.root.mainloop()



class GUI_Plotter(GUI):
    """
    A GUI app that plots and animates a function

    Attributes:
         amp: Description
         freq: Description
         x: Description
         y:
         func:
         plotter_main:
         plotter_sec:
         sec_frame:
         started:
         animation:
         paused:

    Methods:
         main: A wrapper for root.mainloop
         pause:
         start:
         update:
         create_canvas:
         hide_frames:
         show_main:
         show_sec:
         plot:

    """

    def __init__(self):
        super().__init__()
        self.amp = -5
        self.freq = 3
        self.x = []
        self.y = []
        self.func = lambda t : 3 * np.pi * np.exp(self.amp*np.sin(self.freq*np.pi*t))
        self.plotter_main = Plotter(self.func)
        self.plotter_sec = Plotter(self.func)
        self.sec_frame = self.add_frame(bg='Green')
        self.started = False
        self.animation = None
        self.paused = False

        # UI
        self.add_entry_box('Amplitude', self.main_frame)
        self.add_entry_box('Amplitude', self.sec_frame)
        self.add_entry_box('Frequency', self.main_frame)
        self.add_entry_box('Frequency', self.sec_frame)
        self.add_button('Plot', self.plot, self.main_frame)
        self.add_button('Start', self.start, self.sec_frame)
        self.add_button('Pause', self.pause, self.sec_frame)
        self.add_menu_item(label='Exercise 2', command=self.show_main)
        self.add_menu_item(label='Exercise 2.1', command=self.show_sec, last_item=True)
        self.show_main()
        self.create_canvas(self.main_frame, self.plotter_main)
        self.create_canvas(self.sec_frame, self.plotter_sec)

    def main(self):
        """A wrapper for root.mainloop"""
        self.root.mainloop()

    def pause(self):
        """Pauses the animation if paused, starts it if not"""
        if self.started:
            if not self.paused:
                self.animation.event_source.stop()
            else:
                self.animation.event_source.start()
            self.paused = not self.paused

    def start(self):
        """Starts the animation and runs the tkinter mainloop"""
        self.started = True
        self.update(self.plotter_sec)
        self.animation = self.plotter_sec.get_animation()
        self.root.mainloop()

    def update(self, plotter_obj):
        """A simle helper function that updates the values and sets limits based on those"""
        self.amp = float(self.entry_boxes['Amplitude'].get())
        self.freq = float(self.entry_boxes['Frequency'].get())
        plotter_obj.axes.set_xlim([0, 1000])
        starting_val = self.func(1)
        print(starting_val)
        plotter_obj.axes.set_ylim([starting_val-starting_val/(10**12),
                                         starting_val+starting_val/(10**12)])

    def create_canvas(self, frame, plotter_obj):
        """A helper function that sets up a canvas where the plot is drawn"""
        canvas = FigureCanvasTkAgg(plotter_obj.figure, master=frame)
        canvas.draw()
        canvas.get_tk_widget().pack(side=BOTTOM, fill=BOTH, expand=1)

    def hide_frames(self):
        """A helper function for reseting the frames when switching"""
        self.main_frame.pack_forget()
        self.sec_frame.pack_forget()

    def show_main(self):
        """Shows the main frame"""
        self.hide_frames()
        self.main_frame.pack(fill=BOTH, expand=1)

    def show_sec(self):
        """Shows the second frame"""
        self.hide_frames()
        self.sec_frame.pack(fill=BOTH, expand=1)

    def plot(self):
        """Plots data inside the canvas tkinter widget"""
        self.update(self.plotter_main)
        self.plotter_main.plot(1000)





g = GUI_Plotter()
g.main()
