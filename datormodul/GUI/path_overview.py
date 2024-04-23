import tkinter as tk
import information
from PIL import Image, ImageTk

class PathOverview(tk.Frame):

    def __init__(self, parent):
        self.drawn_cones = []
        self.grid_lines = []
        self.bezier_curve_lines = []
        self.waypoints = []
        self.grid_width = 50
        self.zoom_level = 1
        
        path_overview = tk.Frame(parent)
        path_overview.place(x=0, y=0, height=400, width=500)

        self.canvas = tk.Canvas(path_overview, height=400, width=500, bg="#E6E6E6", borderwidth=0, highlightthickness=0)
        self.canvas.place(x=0, y=0)
        
        self.create_grid(self.grid_width)
        self.grid_label = tk.Label(path_overview, text=str(self.grid_width)+" cm")
        self.grid_label.place(x=0, y=400, anchor="sw")
        
        self.source_image = Image.open("racecar_low.png")
        self.car = ImageTk.PhotoImage(self.source_image)

        self.rorry = self.canvas.create_image(250, 200, image=self.car)

        parent.bind("<Key>", self.key_pressed)

    def key_pressed(self, event):
        if event.keysym == "plus" and str(event.widget.focus_get()) != ".terminal.input":
            self.zoom_level += 0.1
            self.change_zoom(self.zoom_level)
        elif event.keysym == "minus" and str(event.widget.focus_get()) != ".terminal.input":
            self.zoom_level -= 0.1
            self.change_zoom(self.zoom_level)
        return

    def change_zoom(self, zoom_level):
        self.zoom_level = zoom_level
        if self.zoom_level < 0.1:
                self.zoom_level = 0.1
        self.change_grid(self.grid_width)
        image = self.source_image.resize((int(self.source_image.width * self.zoom_level), 
                                          int(self.source_image.height * self.zoom_level)))
        self.car = ImageTk.PhotoImage(image)
        self.canvas.delete(self.rorry)
        self.rorry = self.canvas.create_image(250, 200, image=self.car)
        return

    def change_grid(self, width):
        self.grid_width = width
        self.create_grid(width * self.zoom_level)
        self.grid_label.config(text=str(width)+" cm")

    def draw_waypoint(self):
        for waypoint in self.waypoints:
            self.canvas.delete(waypoint)
        self.waypoints.clear()

        for point in information.bezier[1]:
            self.waypoints.append(self.create_circle(point[0], point[1], 3/self.zoom_level, "green"))
        return

    def draw_bezier_curve(self, p1_tuple, p2_tuple, p3_tuple, p4_tuple):
        control_points = [(p1_tuple[0]*self.zoom_level+250, -p1_tuple[1]*self.zoom_level+200), 
                          (p2_tuple[0]*self.zoom_level+250, -p2_tuple[1]*self.zoom_level+200), 
                          (p3_tuple[0]*self.zoom_level+250, -p3_tuple[1]*self.zoom_level+200), 
                          (p4_tuple[0]*self.zoom_level+250, -p4_tuple[1]*self.zoom_level+200)]

        for line in self.bezier_curve_lines:
            self.canvas.delete(line)
        self.bezier_curve_lines.clear()

        x_start = control_points[0][0]
        y_start = control_points[0][1]

        p = control_points

        # loops through
        n = 50
        for i in range(n):
            t = (i+1) / n
            x = (p[0][0] * (1-t)**3 + p[1][0] * 3 * t * (1-t)**2 + p[2][0] * 3 * t**2 * (1-t) + p[3][0] * t**3)
            y = (p[0][1] * (1-t)**3 + p[1][1] * 3 * t * (1-t)**2 + p[2][1] * 3 * t**2 * (1-t) + p[3][1] * t**3)

            self.bezier_curve_lines.append(self.canvas.create_line(x, y, x_start, y_start))

            # updates initial values
            x_start = x
            y_start = y
        
        self.bezier_curve_lines.append(self.create_circle(p1_tuple[0], p1_tuple[1], 3/self.zoom_level, "red"))
        self.bezier_curve_lines.append(self.create_circle(p2_tuple[0], p2_tuple[1], 3/self.zoom_level, "red"))
        self.bezier_curve_lines.append(self.create_circle(p3_tuple[0], p3_tuple[1], 3/self.zoom_level, "red"))
        self.bezier_curve_lines.append(self.create_circle(p4_tuple[0], p4_tuple[1], 3/self.zoom_level, "red"))
        return

    def create_circle(self, x, y, r, color="black"):
        x = x * self.zoom_level
        y = y * self.zoom_level
        r = r * self.zoom_level
        return self.canvas.create_oval(x-r+250, -y-r+200, x+r+250, -y+r+200, fill=color)
    
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
        
        self.draw_waypoint()

        self.draw_bezier_curve(information.bezier[0][0], 
                               information.bezier[0][1], 
                               information.bezier[0][2], 
                               information.bezier[0][3])
