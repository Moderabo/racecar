import tkinter as tk

import information

# Define widget showing data from the car.
class DataOverview(tk.Frame):

    def __init__(self, parent):
        # Set up the widget.
        self.parent = parent
        info_widget = tk.Frame(parent, bg="green", borderwidth=0, highlightthickness=0)
        info_widget.place(x=500, y=0, height=300, width=300)
        
        text_area_info = tk.Message(info_widget, font=("Times New Roman", 15), justify="right", anchor="ne", bg="white", borderwidth=0, highlightthickness=0)
        text_area_info.place(x=0, y=0, height=300, width=150)
        
        # SHow the text in the widget.
        text_value = ""
        for key in information.data.keys():
            text_value += key + "\n"
        text_area_info.config(text=text_value)

        self.text_area_values = tk.Message(info_widget, font=("Times New Roman", 15), justify="left", anchor="nw", bg="white", borderwidth=0, highlightthickness=0)
        self.text_area_values.place(x=150, y=0, height=300, width=150)

    # Update the numerical data shown.
    def update(self):
        text_value = ""
        for value in information.data.values():
            text_value += str(value) + "\n"
        self.text_area_values.config(text=text_value)
