import tkinter as tk
import information

class Terminal(tk.Frame):

    def __init__(self, parent):
        self.parent = parent
        terminal = tk.Frame(parent, height=200, width=500, bg="gray", borderwidth=0, highlightthickness=0)
        terminal.place(x=0, y=400)

        text_area_terminal = tk.Text(terminal, font=("Times New Roman", 15), state="normal", borderwidth=0, highlightthickness=0)
        text_area_terminal.place(x=0, y=0, width=500, height=150)
        text_area_terminal.insert("insert", "\n\n\n\n\n")
        text_area_terminal.config(state="disabled")
        
        input_area = tk.Entry(terminal, bg="gray", font=("Times New Roman", 15), borderwidth=0, highlightthickness=0)
        input_area.place(x=0, y=150, width=500, height=50)
        input_area.bind("<Return>", lambda event : self.command_input(input_area, text_area_terminal))

    def update():
        return

    def command_input(self, usr_input, terminal):
        command = usr_input.get()
        command_list = command.split(" ")

        if command_list[0] == "exit":
            print("TERMINATING")
            self.parent.destroy()
            return
        elif command_list[0] == "h" and len(command_list) > 1:
            information.data.update({"Hastighet" : float(command_list[1])})
            print(information.data)
        elif command_list[0] == "mode" and len(command_list) > 1:
            self.parent.change_driving_mode(command_list[1])
        elif command_list[0] == "stop":
            self.parent.change_driving_mode("stop")
        elif command_list[0] == "start":
            self.parent.mqtt_client.loop_start()
        elif command_list[0] == "end":
            self.parent.is_in = False

        print(command)
        usr_input.delete(0, "end")
        terminal.config(state="normal")
        terminal.insert("end", "\n" + command)
        terminal.config(state="disabled")
        terminal.see("end")