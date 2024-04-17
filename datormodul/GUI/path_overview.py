import tkinter as tk
import information

class PathOverview(tk.Frame):

    def __init__(self, parent):
        path_overview = tk.Frame(parent)
        path_overview.place(x=0, y=0, height=400, width=500)
        self.canvas = tk.Canvas(path_overview, height=400, width=500, bg="#E6E6E6", borderwidth=0, highlightthickness=0)
        self.canvas.place(x=0, y=0)
        self.drawn_cones = []
        self.create_cross(0, 0, 20)

    def create_circle(self, x, y, r):
        return self.canvas.create_oval(x-r+250, -y-r+200, x+r+250, -y+r+200, fill="black")

    def create_cross(self, x, y, d):
        self.canvas.create_line(x-d/2+250, -y-d/2+200, x+d/2+250, -y+d/2+200, width=3, fill="red")
        self.canvas.create_line(x-d/2+250, -y+d/2+200, x+d/2+250, -y-d/2+200, width=3, fill="red")

    def update(self):
        for old_cone in self.drawn_cones:
            self.canvas.delete(old_cone)
        
        for new_cone in information.cones:
            self.drawn_cones.append(self.create_circle(new_cone[0], new_cone[1], new_cone[2]))
