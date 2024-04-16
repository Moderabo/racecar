import tkinter as tk

import information

class DataOverview(tk.Frame):

    def __init__(self, parent):
        self.parent = parent
        info_widget = tk.Frame(parent, bg="green", borderwidth=0, highlightthickness=0)
        info_widget.place(x=500, y=0, height=600, width=300)
        
        text_area_info = tk.Message(info_widget, font=("Times New Roman", 15), justify="right", anchor="ne", bg="white", borderwidth=0, highlightthickness=0)
        text_area_info.place(x=0, y=0, height=400, width=150)
        
        text_value = ""
        for key in information.data.keys():
            text_value += str(key) + "\n"
        text_area_info.config(text=text_value)

        self.text_area_values = tk.Message(info_widget, font=("Times New Roman", 15), justify="left", anchor="nw", bg="white", borderwidth=0, highlightthickness=0)
        self.text_area_values.place(x=150, y=0, height=400, width=150)

    def update(self):
        text_value = ""
        for value in information.data.values():
            text_value += str(value) + "\n"
        text_value += str(information.data_list[-1])
        self.text_area_values.config(text=text_value)
