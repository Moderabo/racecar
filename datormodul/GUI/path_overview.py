import tkinter as tk
import information

class PathOverview(tk.Frame):

    def __init__(self, parent):
        self.drawn_cones = []
        self.grid_lines = []
        
        path_overview = tk.Frame(parent)
        path_overview.place(x=0, y=0, height=400, width=500)

        self.canvas = tk.Canvas(path_overview, height=400, width=500, bg="#E6E6E6", borderwidth=0, highlightthickness=0)
        self.canvas.place(x=0, y=0)
        
        self.create_grid(50)
        self.grid_label = tk.Label(path_overview, text="50 cm")
        self.grid_label.place(x=0, y=400, anchor="sw")
        
        self.car = tk.PhotoImage(file="racecar_low.png")
        self.canvas.create_image(250,200, image=self.car)

    def change_grid(self, width):
        self.create_grid(width)
        self.grid_label.config(text=str(width)+" cm")


    def create_circle(self, x, y, r):
        return self.canvas.create_oval(x-r+250, -y-r+200, x+r+250, -y+r+200, fill="black")
    
    def create_grid(self, width):
        for line in self.grid_lines:
            self.canvas.delete(line)

        x = 250
        while x < 500:
            self.grid_lines.append(self.canvas.create_line(x, 0, x, 400, fill="gray"))
            self.grid_lines.append(self.canvas.create_line(500-x, 0, 500-x, 400,fill="gray"))
            x += width
        
        y = 200
        while y < 400:
            self.grid_lines.append(self.canvas.create_line(0, y, 500, y, fill="gray"))
            self.grid_lines.append(self.canvas.create_line(0, 400-y, 500, 400-y, fill="gray"))
            y += width


    def create_cross(self, x, y, d):
        self.canvas.create_line(x-d/2+250, -y-d/2+200, x+d/2+250, -y+d/2+200, width=3, fill="red")
        self.canvas.create_line(x-d/2+250, -y+d/2+200, x+d/2+250, -y-d/2+200, width=3, fill="red")

    def update(self):
        for old_cone in self.drawn_cones:
            self.canvas.delete(old_cone)
        
        for new_cone in information.cones:
            self.drawn_cones.append(self.create_circle(new_cone[0], new_cone[1], new_cone[2]))
