import tkinter as tk

import information

class Interactions(tk.Frame):

    def __init__(self, parent):
        self.parent = parent
        interaction_widget = tk.Frame(parent, bg="green", borderwidth=0, highlightthickness=0)
        interaction_widget.place(x=500, y=300, height=300, width=300)
        interaction_widget.bind("<Leave>", lambda event: self.change_text("CarGoBRR2"))

        #=====Parameter for auto-driving widget=====
        auto_param_widget = tk.Frame(interaction_widget, bg="blue", borderwidth=0, highlightthickness=0)
        auto_param_widget.place(x=0, y=0, height=30, width=300)
        
        self.parameter_entries = []
        parameter_explanation = ["Largest gas. (0-1)", "Lowest gas. (0-1)", "Lowest turning radius in mm. (min. 800 mm)", 
                                 "Largest turning radius in mm.", "Control parameter for angle to goal-point.", 
                                 "Control parameter for offset to line."]
        self.parameters = []

        auto_param_widget_width = 300

        for i in range(6):
            self.parameters.append(tk.DoubleVar())
            self.parameters[i].set(information.parameters[i])
            parameter_entry = tk.Entry(auto_param_widget, bg="#E6E6E6", borderwidth=0, highlightthickness=0, 
                                       bd=3, textvariable=self.parameters[i])
            parameter_entry.place(x=0+i*auto_param_widget_width/6, y=0, width=auto_param_widget_width/6, height=30)
            self.parameter_entries.append(parameter_entry)
            parameter_entry.bind("<Enter>", lambda event: self.change_text(parameter_explanation[i]))

        self.parameter_entries[0].bind("<Enter>", lambda event: self.change_text(parameter_explanation[0]))
        self.parameter_entries[1].bind("<Enter>", lambda event: self.change_text(parameter_explanation[1]))
        self.parameter_entries[2].bind("<Enter>", lambda event: self.change_text(parameter_explanation[2]))
        self.parameter_entries[3].bind("<Enter>", lambda event: self.change_text(parameter_explanation[3]))
        self.parameter_entries[4].bind("<Enter>", lambda event: self.change_text(parameter_explanation[4]))
        self.parameter_entries[5].bind("<Enter>", lambda event: self.change_text(parameter_explanation[5]))
        #============================================

        #=====Change driving mode widget=====
        self.current_drive_mode = ""

        driving_mode_widget = tk.Frame(interaction_widget, bg="yellow", borderwidth=0, highlightthickness=0)
        driving_mode_widget.place(x=0, y=30, height=30, width=300)

        self.idle_button = tk.Button(driving_mode_widget, bg="#E6E6E6", text="Idle", 
                                    command=lambda: self.new_command("idle"), 
                                    borderwidth=0, highlightthickness=0, bd=3)
        self.idle_button.place(x=0, y=0, height=30, width=75)

        self.auto_button = tk.Button(driving_mode_widget, bg="#E6E6E6", text="Auto",
                                     command=lambda: self.new_command("auto"), 
                                     borderwidth=0, highlightthickness=0, bd=3)
        self.auto_button.place(x=75, y=0, height=30, width=75)

        self.manual_button = tk.Button(driving_mode_widget, bg="#E6E6E6", text="Manual", 
                                       command=lambda: self.new_command("manual"),
                                       borderwidth=0, highlightthickness=0, bd=3)
        self.manual_button.place(x=150, y=0, height=30, width=75)

        self.wasd_button = tk.Button(driving_mode_widget, bg="#E6E6E6", text="WASD", 
                                     command=lambda: self.new_command("wasd"), 
                                     borderwidth=0, highlightthickness=0, bd=3)
        self.wasd_button.place(x=225, y=0, height=30, width=75)

        self.idle_button.bind("<Enter>", lambda event: self.change_text("Driving mode: Idle \nThe car will stop and stand idle."))
        self.auto_button.bind("<Enter>", lambda event: self.change_text("Driving mode: Auto \nThe car will automatically drive through the course using the given parameters above."))
        self.manual_button.bind("<Enter>", lambda event: self.change_text("Driving mode: Manual \nThe car will take driving commands from the plugged in controller."))
        self.wasd_button.bind("<Enter>", lambda event: self.change_text("Driving mode: WASD \nThe car will take driving commands from the keyboard using the w, a, s and d keys as input."))
        #=====================================

        #=====Explanation widget=====
        self.explanation_widget = tk.Message(interaction_widget, bg="#E6E6E6", text="Welcome to my terminal!", width=300, justify="center")
        self.explanation_widget.place(x=0, y=60, height=100, width=300)
        self.explanation_widget.bind("<Enter>", lambda event: self.change_text("Tip: \nHover over a button or entry to get an explanation of it."))
        #============================

        #=====Save data widget=====
        record_widget = tk.Frame(interaction_widget, bg="white", borderwidth=0, highlightthickness=0)
        record_widget.place(x=0, y=160, height=40, width=300)

        start_record_button = tk.Button(record_widget, bg="#E6E6E6", text="Begin recording", 
                                        command=self.parent.record_drive_data, 
                                        borderwidth=0, highlightthickness=0, bd=3)
        start_record_button.place(x=0, y=0, height=40, width=150)

        stop_record_button = tk.Button(record_widget, bg="#E6E6E6", text="Stop recording", 
                                       command=self.parent.stop_drive_data, 
                                       borderwidth=0, highlightthickness=0, bd=3)
        stop_record_button.place(x=150, y=0, height=40, width=150)

        start_record_button.bind("<Enter>", lambda event: self.change_text("Begin recording all the information sent between the car and the computer to file."))
        stop_record_button.bind("<Enter>", lambda event: self.change_text("Stop the recording and save the file."))
        #==========================

        #=====Controls=====
        control_widget = tk.Frame(interaction_widget, bg="blue", borderwidth=0, highlightthickness=0)
        control_widget.place(x=0, y=200, height=40, width=300)

        setup_button = tk.Button(control_widget, bg="#E6E6E6", text="Setup", 
                                 command=self.setup, 
                                 borderwidth=0, highlightthickness=0, bd=3)
        setup_button.place(x=0, y=0, height=40, width=100)

        stop_button = tk.Button(control_widget, bg="#E6E6E6", text="Stop", 
                                command=lambda:self.new_command("stop"), 
                                borderwidth=0, highlightthickness=0, bd=3)
        stop_button.place(x=100, y=0, height=40, width=100)

        reset_button = tk.Button(control_widget, bg="#E6E6E6", text="Reset", 
                                 command=self.reset, 
                                 borderwidth=0, highlightthickness=0, bd=3)
        reset_button.place(x=200, y=0, height=40, width=100)

        setup_button.bind("<Enter>", lambda event: self.change_text("Connect to the car. \n(Will take a few seconds and freeze the GUI)"))
        stop_button.bind("<Enter>", lambda event: self.change_text("Completely stop sending information between the car and the computer."))
        reset_button.bind("<Enter>", lambda event: self.change_text("Reset the interactive map and the auto-parameters."))
        #==================

        #=====Exit=====
        exit_button = tk.Button(interaction_widget, bg="red", text="Exit", 
                                command=self.parent.exit, 
                                borderwidth=0, highlightthickness=0, bd=3)
        exit_button.place(x=0, y=240, height=60, width=300)

        exit_button.bind("<Enter>", lambda event: self.change_text("Exit and close the GUI."))
        #==============

    def reset(self):
        information.parameters = information.default_parameters
        for i in range(6):
            self.parameters[i].set(information.parameters[i])
        self.parent.path_widget.reset()

    def new_command(self, mode):
        if mode == "manual" and self.parent.controller.controller_exists() == False:
            self.change_text("No controller detected.")
            return
        try:
            self.parent.change_driving_mode(mode)
        except:
            self.parent.change_driving_mode("stop")
            self.change_text("ERROR.")
            return
        self.current_drive_mode = mode
    def setup(self):
        try:
            self.parent.setup_mqtt_protocol()
            self.explanation_widget.configure(text="Connected to car.")
        except:
            self.explanation_widget.configure(text="Could not connect to car.")

    def change_text(self, text):
        self.explanation_widget.configure(text=text)

    def update(self):
        # Update information for the parameters.
        for i in range(6):
            try:
                information.parameters[i] = self.parameters[i].get()
                self.parameter_entries[i].configure(bg="#E6E6E6")
            except:
                self.parameter_entries[i].configure(bg="red")
                continue
        
        self.idle_button.configure(bg="#E6E6E6")
        self.auto_button.configure(bg="#E6E6E6")
        self.manual_button.configure(bg="#E6E6E6")
        self.wasd_button.configure(bg="#E6E6E6")

        if self.current_drive_mode == "idle":
            self.idle_button.configure(bg="gray")
            #self.auto_button.configure(bg="#E6E6E6")
            #self.manual_button.configure(bg="#E6E6E6")
            #self.wasd_button.configure(bg="#E6E6E6")
        elif self.current_drive_mode == "auto":
            #self.idle_button.configure(bg="#E6E6E6")
            self.auto_button.configure(bg="gray")
            #self.manual_button.configure(bg="#E6E6E6")
            #self.wasd_button.configure(bg="#E6E6E6")
        elif self.current_drive_mode == "manual":
            #self.idle_button.configure(bg="#E6E6E6")
            #self.auto_button.configure(bg="#E6E6E6")
            self.manual_button.configure(bg="gray")
            #self.wasd_button.configure(bg="#E6E6E6")
        elif self.current_drive_mode == "wasd":
            #self.idle_button.configure(bg="#E6E6E6")
            #self.auto_button.configure(bg="#E6E6E6")
            #self.manual_button.configure(bg="#E6E6E6")
            self.wasd_button.configure(bg="gray")
        #else:
            #self.idle_button.configure(bg="#E6E6E6")
            #self.auto_button.configure(bg="#E6E6E6")
            #self.manual_button.configure(bg="#E6E6E6")
            #self.wasd_button.configure(bg="#E6E6E6")
            
        return
