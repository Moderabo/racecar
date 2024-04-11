import tkinter as tk

class PathOverview(tk.Frame):

    def __init__(self, parent):
        path_overview = tk.Frame(parent)
        path_overview.place(x=0, y=0, height=400, width=500)
        self.canvas = tk.Canvas(path_overview, height=400, width=500, bg="#E6E6E6", borderwidth=0, highlightthickness=0)
        self.canvas.place(x=0, y=0)