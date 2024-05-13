import tkinter as tk
import information

class Terminal(tk.Frame):

    def __init__(self, parent):
        self.parent = parent
        terminal = tk.Frame(parent, height=200, width=500, bg="gray", borderwidth=0, highlightthickness=0, name="terminal")
        terminal.place(x=0, y=400)

        #=====Set up terminal text area=====
        self.text_area_terminal = tk.Text(terminal, font=("Times New Roman", 15), state="normal", borderwidth=0, highlightthickness=0)
        self.text_area_terminal.place(x=0, y=0, width=500, height=150)
        self.text_area_terminal.insert("insert", "\n\n\n\n\n")
        self.text_area_terminal.config(state="disabled")
        #===================================
        
        #=====Set up terminal input=====
        input_area = tk.Entry(terminal, bg="gray", font=("Times New Roman", 15), borderwidth=0, highlightthickness=0, name="input")
        input_area.place(x=0, y=150, width=500, height=50)
        input_area.bind("<Return>", lambda event : self.command_input(input_area, self.text_area_terminal))
        #===============================

    def command_input(self, usr_input, terminal):
        # Read and perform correct action from the command. Function is run
        # every time Enter is pressed.

        # Read the input.
        command = usr_input.get()
        command_list = command.split(" ")
        info_to_print = ""

        # Check command and perform correct action
        if command_list[0] == "exit":
            self.parent.exit()
            return
        elif command_list[0] == "mode" and len(command_list) > 1:
            if len(command_list) > 7 and command_list[1] == "auto":
                argument_list = []
                for argument in command_list[2:8]:
                    argument_list.append(argument)
                argument_list = tuple(argument_list)
                self.parent.change_driving_mode(command_list[1], argument_list)
            else:
                self.parent.change_driving_mode(command_list[1])
        elif command_list[0] == "stop":
            self.parent.change_driving_mode("stop")
        elif command_list[0] == "start":
            self.parent.change_driving_mode("idle")
        elif command_list[0] == "end":
            self.parent.is_in = False
        elif command_list[0] == "cone" and len(command_list) > 3:
            information.cones.append((float(command_list[1]), float(command_list[2]), float(command_list[3])))
        elif command_list[0] == "cones":
            info_to_print = str(information.cones)
        elif command_list[0] == "rmvcone" and len(command_list) > 1:
            information.cones.pop(int(command_list[1]))
        elif command_list[0] == "grid" and len(command_list) > 1:
            self.parent.path_widget.change_grid(int(command_list[1]))
        elif command_list[0] == "zoom" and len(command_list) > 1:
            self.parent.path_widget.change_zoom(float(command_list[1]))
        elif command_list[0] == "reset":
            self.parent.path_widget.reset()
        elif command == "record start":
            self.parent.record_drive_data()
        elif command == "record stop":
            self.parent.stop_drive_data()
        elif command_list[0] == "graph":
            self.parent.create_graph(command_list[1])
        elif command_list[0] == "setup":
            try:
                self.parent.setup_mqtt_protocol()
            except:
                info_to_print = "Could not connect to car."
        else:
            usr_input.delete(0, "end")
            terminal.config(state="normal")
            terminal.insert("end", "\nCommand not found: " + command)
            terminal.config(state="disabled")
            terminal.see("end")
            return

        # Write the command to the terminal text window and clear the input.
        usr_input.delete(0, "end")
        terminal.config(state="normal")
        terminal.insert("end", "\n" + command)
        if info_to_print != "":
            terminal.insert("end", "\n" + info_to_print)
        terminal.config(state="disabled")
        terminal.see("end")
