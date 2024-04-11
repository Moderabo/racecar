import tkinter as tk
import path_overview as path_overview
import terminal
import data_overview
import information
import mqt

class GUI(tk.Tk):

    def __init__(self, title):
        #=====Set up main window=====
        width = 800
        height = 600
        width_offset = 0
        height_offset = 0
        super().__init__()
        self.geometry("%dx%d+%d+%d"%(width, height, width_offset, height_offset))
        self.resizable(False, False)
        self.title(title)
        self.configure(bg="white")
        #============================

        #=====Set up the widgets=====
        self.path_widget = path_overview.PathOverview(self)
        self.terminal_widget = terminal.Terminal(self)
        self.data_widget = data_overview.DataOverview(self)
        #============================

        self.mqttc = mqt.mqttc

        self.update_GUI()

    def __del__(self):
        return

    def change_driving_mode(self):
        return

    def update_GUI(self): 
        text_value = ""
        for value in information.data.values():
            text_value += str(value) + "\n"
        text_value += str(mqt.data_list[-1])
        self.data_widget.text_area_values.config(text=text_value)
        self.after(100, self.update_GUI)


UI = GUI("CarGoBrr2")
UI.mainloop()
