import tkinter as tk
import os
import time
import path_overview
import terminal
import data_overview
import mqtt
import xbox
import read_keyboard
import paho.mqtt.client as client
import information
import interactions
from datetime import datetime


def clamp(minimum, x, maximum):
    return max(minimum, min(x, maximum))

class GUI(tk.Tk):

    def __init__(self, title):

        #=====Set up main window=====
        width = 800
        height = 600
        width_offset = 0
        height_offset = 0
        super().__init__()
        self.geometry("%dx%d+%d+%d"%(width, height, width_offset, height_offset))
        self.resizable(False, False)
        self.title(title)
        self.configure(bg="white")
        #============================

        #=====Set up the widgets=====
        self.path_widget = path_overview.PathOverview(self)
        self.terminal_widget = terminal.Terminal(self)
        self.data_widget = data_overview.DataOverview(self)
        self.input_widget = interactions.Interactions(self)
        #============================

        #=====Set up MQTT client=====
        self.mqtt_commands = mqtt.Mqtt()
        self.unacked_publish = set()
        self.mqtt_client = client.Client(client.CallbackAPIVersion.VERSION2)
        self.mqtt_client.on_connect = self.mqtt_commands.on_connect
        self.mqtt_client.on_message = self.mqtt_commands.on_message
        self.mqtt_client.on_subscribe = self.mqtt_commands.on_subscribe
        self.mqtt_client.on_unsubscribe = self.mqtt_commands.on_unsubscribe
        self.mqtt_client.on_publish = self.mqtt_commands.on_publish
        self.mqtt_client.user_data_set(self.unacked_publish)
        #=============================

        try:
            self.controller = xbox.XboxController()
        except:
            print("No controller detected!")
        
        self.steering_mode = "stop"
        self.currently_writing_to_file = False

        self.update_GUI()
        self.is_on = True
        self.steering()

        self.kbd = read_keyboard.Keyboard()

        self.bind("<Button-1>", self.click_event)

        self.date_format_file = ""

    def __del__(self):
        os.system(f'''cmd /c "netsh wlan connect name=eduroam"''')
        return

    def exit(self):
        print("TERMINATING")
        try:
            self.change_driving_mode("stop")
        except:
            print("Not connected")
        self.stop_drive_data()
        self.destroy()


    def stop_drive_data(self):
        if self.currently_writing_to_file == False:
            self.terminal_widget.text_area_terminal.config(state="normal")
            self.terminal_widget.text_area_terminal.insert("end", "\nError: Nothing to stop.")
            self.terminal_widget.text_area_terminal.config(state="disabled")
            self.terminal_widget.text_area_terminal.see("end")
            return
        self.currently_writing_to_file = False
        self.mqtt_commands.close_file()

        self.terminal_widget.text_area_terminal.config(state="normal")
        self.terminal_widget.text_area_terminal.insert("end", f"\nStopped recording. File saved as {self.date_format_file}.txt" )
        self.terminal_widget.text_area_terminal.config(state="disabled")
        self.terminal_widget.text_area_terminal.see("end")
        return

    def record_drive_data(self):
        if self.currently_writing_to_file == True:
            self.terminal_widget.text_area_terminal.config(state="normal")
            self.terminal_widget.text_area_terminal.insert("end", "\nError: Already recording.")
            self.terminal_widget.text_area_terminal.config(state="disabled")
            self.terminal_widget.text_area_terminal.see("end")
            return
        
        self.currently_writing_to_file = True
        now = datetime.now()
        self.date_format_file = now.strftime("%Y-%m-%d_%H-%M-%S")
        self.mqtt_commands.create_file(self.date_format_file + ".txt")

        date_format_terminal = now.strftime("%Y-%m-%d %H:%M:%S")
        self.terminal_widget.text_area_terminal.config(state="normal")
        self.terminal_widget.text_area_terminal.insert("end", f"\nBegan recording at {date_format_terminal}." )
        self.terminal_widget.text_area_terminal.config(state="disabled")
        self.terminal_widget.text_area_terminal.see("end")
        return

    def setup_mqtt_protocol(self):
        os.system(f'''cmd /c "netsh wlan connect name=CarGoBRR2"''')
        time.sleep(5)
        self.mqtt_client.connect("10.42.0.1")

    def click_event(self, event):
        event.widget.focus_set()

    def change_driving_mode(self, mode, parameters = information.parameters):
        if self.steering_mode == "stop":
            self.mqtt_client.loop_start()
        self.steering_mode = mode
        
        time.sleep(0.3)
        msg_info = self.mqtt_client.publish("commands", "0", qos=1)
        self.unacked_publish.add(msg_info.mid)
        msg_info.wait_for_publish()
        time.sleep(0.2)

        if mode == "auto":
            parameter_string = ""
            for parameter in parameters:
                parameter_string += " " + str(parameter)

            msg_info = self.mqtt_client.publish("commands", "2" + parameter_string, qos=1)
            self.unacked_publish.add(msg_info.mid)
            msg_info.wait_for_publish()

        return

    def update_GUI(self): 
        self.data_widget.update()
        self.path_widget.update()
        self.input_widget.update()
        self.after(10, self.update_GUI)

    def steering(self):
        match self.steering_mode:
            case "manual":
                controller_inputs = self.controller.read()
                x_cord = controller_inputs[0]
                rt = controller_inputs[4]
                lt = controller_inputs[5]

                servo_signal = x_cord
                motor = rt * 0.5 - lt * 0.5

                if -0.1 <= abs(servo_signal) <= 0.1:
                    servo_signal = 0.0
                else:
                    servo_signal = clamp(-1, servo_signal, 1)

                motor = clamp(-1.0, motor, 1.0)

                msg_info = self.mqtt_client.publish("commands", "1 " + str(servo_signal) + " "+ str(motor), qos=1)
                self.unacked_publish.add(msg_info.mid)

                msg_info.wait_for_publish()

            case "wasd":
                servo_signal = 0
                motor = 0
                kbd_input = self.kbd.read()
                if kbd_input[0] == 1:
                    motor = 0.3
                elif kbd_input[2] == 1:
                    motor = -0.3
                if kbd_input[1] == 1:
                    servo_signal = -1
                elif kbd_input[3] == 1:
                    servo_signal = 1
                
                if str(self.focus_get()) == ".terminal.input":
                    motor = 0
                    servo_signal = 0

                msg_info = self.mqtt_client.publish("commands", "1 " + str(servo_signal) + " "+ str(motor), qos=1)
                self.unacked_publish.add(msg_info.mid)

                msg_info.wait_for_publish()

            case "stop":
                self.mqtt_client.loop_stop()
        self.after(100, self.steering)
        return

UI = GUI("CarGoBrr2")
UI.mainloop()
