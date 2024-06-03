import tkinter as tk
import information
from PIL import Image, ImageTk

# Class to give an overview of what the car "sees".
class PathOverview(tk.Frame):

    def __init__(self, parent):
        self.drawn_cones = []
        self.grid_lines = []
        self.bezier_curve_lines = []
        self.waypoints = []
        self.grid_width = 50
        self.zoom_level = 1
        self.x_offset = 0
        self.y_offset = 0
        
        path_overview = tk.Frame(parent, name="path_overview")
        path_overview.place(x=0, y=0, height=400, width=500)

        # Set up the "map" of the surrounding.
        self.canvas = tk.Canvas(path_overview, height=400, width=500, bg="#E6E6E6", 
                                borderwidth=0, highlightthickness=0, name="canvas")
        self.canvas.place(x=0, y=0)
        
        self.create_grid(self.grid_width)
        self.grid_label = tk.Label(path_overview, text=str(self.grid_width)+" cm")
        self.grid_label.place(x=0, y=400, anchor="sw")
        
        self.source_image = Image.open("racecar_low.png")
        self.car = ImageTk.PhotoImage(self.source_image)

        self.rorry = self.canvas.create_image(250, 200, image=self.car)

        parent.bind("<Key>", self.key_pressed)

    # Based on the keyboard, do an action on the map.
    def key_pressed(self, event):
        if event.keysym == "plus" and str(event.widget.focus_get()) != ".terminal.input":
            self.zoom_level += 0.1
            self.change_zoom(self.zoom_level)
        elif event.keysym == "minus" and str(event.widget.focus_get()) != ".terminal.input":
            self.zoom_level -= 0.1
            self.change_zoom(self.zoom_level)
        elif event.keysym == "Right" and str(event.widget.focus_get()) == ".path_overview.canvas":
            self.change_center(x=self.x_offset+5, y=self.y_offset)
        elif event.keysym == "Left" and str(event.widget.focus_get()) == ".path_overview.canvas":
            self.change_center(x=self.x_offset-5, y=self.y_offset)
        elif event.keysym == "Down" and str(event.widget.focus_get()) == ".path_overview.canvas":
            self.change_center(x=self.x_offset, y=self.y_offset-5)
        elif event.keysym == "Up" and str(event.widget.focus_get()) == ".path_overview.canvas":
            self.change_center(x=self.x_offset, y=self.y_offset+5)
        elif event.keysym == "c" and str(event.widget.focus_get()) == ".path_overview.canvas":
            self.change_center(0, 0)
        return

    # Reset the widget to default parameters.
    def reset(self):
        self.change_center(0, 0)
        self.change_grid(50)
        self.change_zoom(1)

    # Changes the center so the car is not in the middle.
    def change_center(self, x, y):
        self.x_offset = x
        self.y_offset = y
        self.change_grid(self.grid_width)
        self.canvas.delete(self.rorry)
        self.rorry = self.canvas.create_image(250-self.x_offset, 200+self.y_offset, image=self.car)
        return

    # Zoom in the widget.
    def change_zoom(self, zoom_level):
        self.zoom_level = zoom_level
        if self.zoom_level < 0.1:
                self.zoom_level = 0.1
        self.change_grid(self.grid_width)
        image = self.source_image.resize((int(self.source_image.width * self.zoom_level), 
                                          int(self.source_image.height * self.zoom_level)))
        self.car = ImageTk.PhotoImage(image)
        self.canvas.delete(self.rorry)
        self.rorry = self.canvas.create_image(250-self.x_offset, 200+self.y_offset, image=self.car)
        return

    # Change the distance between the grid-lines for better accuracy.
    def change_grid(self, width):
        self.grid_width = width
        self.create_grid(width * self.zoom_level)
        self.grid_label.config(text=str(width)+" cm")

    # Draw the points on the bezier curve the car "sees".
    def draw_waypoint(self):
        for waypoint in self.waypoints:
            self.canvas.delete(waypoint)
        self.waypoints.clear()

        for point in information.bezier[1]:
            self.waypoints.append(self.create_circle(point[0], point[1], 3/self.zoom_level, "green"))
        return

    # Draw the correct bezier curve from the four bezier points.
    def draw_bezier_curve(self, p1_tuple, p2_tuple, p3_tuple, p4_tuple):
        control_points = [(p1_tuple[0]*self.zoom_level+250-self.x_offset, -p1_tuple[1]*self.zoom_level+200+self.y_offset), 
                          (p2_tuple[0]*self.zoom_level+250-self.x_offset, -p2_tuple[1]*self.zoom_level+200+self.y_offset), 
                          (p3_tuple[0]*self.zoom_level+250-self.x_offset, -p3_tuple[1]*self.zoom_level+200+self.y_offset), 
                          (p4_tuple[0]*self.zoom_level+250-self.x_offset, -p4_tuple[1]*self.zoom_level+200+self.y_offset)]

        for line in self.bezier_curve_lines:
            self.canvas.delete(line)
        self.bezier_curve_lines.clear()

        x_start = control_points[0][0]
        y_start = control_points[0][1]

        p = control_points

        # Loops through
        n = 50
        for i in range(n):
            t = (i+1) / n
            x = (p[0][0] * (1-t)**3 + p[1][0] * 3 * t * (1-t)**2 + p[2][0] * 3 * t**2 * (1-t) + p[3][0] * t**3)
            y = (p[0][1] * (1-t)**3 + p[1][1] * 3 * t * (1-t)**2 + p[2][1] * 3 * t**2 * (1-t) + p[3][1] * t**3)

            self.bezier_curve_lines.append(self.canvas.create_line(x, y, x_start, y_start))

            # Updates initial values
            x_start = x
            y_start = y
        
        self.bezier_curve_lines.append(self.create_circle(p1_tuple[0], p1_tuple[1], 3/self.zoom_level, "red"))
        self.bezier_curve_lines.append(self.create_circle(p2_tuple[0], p2_tuple[1], 3/self.zoom_level, "red"))
        self.bezier_curve_lines.append(self.create_circle(p3_tuple[0], p3_tuple[1], 3/self.zoom_level, "red"))
        self.bezier_curve_lines.append(self.create_circle(p4_tuple[0], p4_tuple[1], 3/self.zoom_level, "red"))
        return

    # Create a circle.
    def create_circle(self, x, y, r, color="black"):
        x = x * self.zoom_level
        y = y * self.zoom_level
        r = r * self.zoom_level
        return self.canvas.create_oval(x-r+250-self.x_offset, -y-r+200+self.y_offset, 
                                       x+r+250-self.x_offset, -y+r+200+self.y_offset, 
                                       fill=color)
    # Create the grid.
    def create_grid(self, width):
        for line in self.grid_lines:
            self.canvas.delete(line)
        self.grid_lines.clear()

        x = (250-self.x_offset) % width
        x_bound = 500

        y = (200+self.y_offset) % width
        y_bound = 400

        while x < x_bound:
            self.grid_lines.append(self.canvas.create_line(x, 0, x, 400, fill="gray"))
            x += width
        
        while y < y_bound:
            self.grid_lines.append(self.canvas.create_line(0, y, 500, y, fill="gray"))
            y += width

    # Create a cross.
    def create_cross(self, x, y, d):
        self.canvas.create_line(x-d/2+250, -y-d/2+200, x+d/2+250, -y+d/2+200, width=3, fill="red")
        self.canvas.create_line(x-d/2+250, -y+d/2+200, x+d/2+250, -y-d/2+200, width=3, fill="red")

    # Update the "map" with new data from the car.
    def update(self):
        for old_cone in self.drawn_cones:
            self.canvas.delete(old_cone)
        self.drawn_cones.clear()
        
        for new_cone in information.cones:
            self.drawn_cones.append(self.create_circle(new_cone[0], new_cone[1], new_cone[2]))
        
        self.draw_waypoint()

        self.draw_bezier_curve(information.bezier[0][0], 
                               information.bezier[0][1], 
                               information.bezier[0][2], 
                               information.bezier[0][3])
