# GUI.py
# Written by Patrik
# 2024-05-10
# Version 1.0

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
import graph
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

        #=====Set up controller and keyboard=====
        self.controller = xbox.XboxController()
        self.kbd = read_keyboard.Keyboard()
        #========================================

        # Initiate variables used later.
        self.steering_mode = "stop"             # Used to show drivingmode.
        self.date_format_file = ""              # Used to store the date for recording data.
        self.graphs = []                        # List with all the currently active graphs.
        self.currently_writing_to_file = False  # Shows if a recording of data is taking place.

        # Initiate the update- and steering loop.
        self.update_GUI()
        self.steering()

        # Used for setting window focus to clicked widget.
        self.bind("<Button-1>", self.click_event)

    def __del__(self):
        # When GUI is deleted, change back wifi to eduroam.
        os.system(f'''cmd /c "netsh wlan connect name=eduroam"''')
        return

    def exit(self):
        # Perform necessary actions when exiting GUI.
        print("TERMINATING")
        try:
            self.change_driving_mode("stop")
        except:
            print("Not connected.")
        self.stop_drive_data()
        self.destroy()


    def stop_drive_data(self):
        # Stop current recording of data and save. 

        if self.currently_writing_to_file == False:
            # Write out information about the recording to the terminal.
            self.terminal_widget.text_area_terminal.config(state="normal")
            self.terminal_widget.text_area_terminal.insert("end", "\nError: Nothing to stop.")
            self.terminal_widget.text_area_terminal.config(state="disabled")
            self.terminal_widget.text_area_terminal.see("end")
            return
        
        self.currently_writing_to_file = False
        self.mqtt_commands.close_file()

        # Write out information about the recording to the terminal.
        self.terminal_widget.text_area_terminal.config(state="normal")
        self.terminal_widget.text_area_terminal.insert("end", f"\nStopped recording. File saved as {self.date_format_file}.txt" )
        self.terminal_widget.text_area_terminal.config(state="disabled")
        self.terminal_widget.text_area_terminal.see("end")
        return

    def record_drive_data(self):
        # Start a recording of data and save to CURRENT_DATE AND_TIME.txt

        if self.currently_writing_to_file == True:
            # Write out information about the recording to the terminal.
            self.terminal_widget.text_area_terminal.config(state="normal")
            self.terminal_widget.text_area_terminal.insert("end", "\nError: Already recording.")
            self.terminal_widget.text_area_terminal.config(state="disabled")
            self.terminal_widget.text_area_terminal.see("end")
            return
        
        self.currently_writing_to_file = True
        now = datetime.now()
        self.date_format_file = now.strftime("%Y-%m-%d_%H-%M-%S")
        self.mqtt_commands.create_file(self.date_format_file + ".txt")

        # Write out information about the recording to the terminal.
        date_format_terminal = now.strftime("%Y-%m-%d %H:%M:%S")
        self.terminal_widget.text_area_terminal.config(state="normal")
        self.terminal_widget.text_area_terminal.insert("end", f"\nBegan recording at {date_format_terminal}." )
        self.terminal_widget.text_area_terminal.config(state="disabled")
        self.terminal_widget.text_area_terminal.see("end")
        return

    def setup_mqtt_protocol(self):
        # Connect to the car's wifi and connect the client to the car.
        os.system(f'''cmd /c "netsh wlan connect name=CarGoBRR2"''')
        time.sleep(5)
        self.mqtt_client.connect("10.42.0.1")

    def click_event(self, event):
        # Set focus to current widget. 
        event.widget.focus_set()

    def change_driving_mode(self, mode, parameters = information.parameters):
        # Change the current driving mode

        # Start the MQTT-loop if going from mode "stop" to anything other.
        if self.steering_mode == "stop":
            self.mqtt_client.loop_start()
        self.steering_mode = mode
        
        # Wait and send an "idle" command to the car to prepare.
        time.sleep(0.3)
        msg_info = self.mqtt_client.publish("commands", "0", qos=1)
        self.unacked_publish.add(msg_info.mid)
        msg_info.wait_for_publish()
        time.sleep(0.2)

        # Send all parameters to the car if driving mode is changed to "auto".
        if mode == "auto":
            parameter_string = ""
            for parameter in parameters:
                parameter_string += " " + str(parameter)
            msg_info = self.mqtt_client.publish("commands", "2" + parameter_string, qos=1)
            self.unacked_publish.add(msg_info.mid)
            msg_info.wait_for_publish()

        return

    def update_GUI(self): 
        # Every 100 ms update the GUI. 
        self.data_widget.update()
        self.path_widget.update()
        self.input_widget.update()
        self.after(100, self.update_GUI)

    def steering(self):
        # Perform necessary calculations and send steering signals to the car
        # if it steering mode is "manual" or "wasd" every 100 ms.

        match self.steering_mode:
            case "manual":

                # Read the controller.
                controller_inputs = self.controller.read()
                x_cord = controller_inputs[0]
                rt = controller_inputs[4]
                lt = controller_inputs[5]

                # Set the steering signals according to the controller.
                servo_signal = x_cord
                motor = rt * 0.5 - lt * 0.5

                # Make sure only accurate signals can be sent.
                if -0.1 <= abs(servo_signal) <= 0.1:            # Have a dead-zone for the joy-stick.
                    servo_signal = 0.0
                else:
                    servo_signal = clamp(-1, servo_signal, 1)   # Clamp the signal.

                motor = clamp(-1.0, motor, 1.0)

                # Send the steering signals
                msg_info = self.mqtt_client.publish("commands", "1 " + str(servo_signal) + " "+ str(motor), qos=1)
                self.unacked_publish.add(msg_info.mid)
                msg_info.wait_for_publish()

            case "wasd":
                servo_signal = 0
                motor = 0

                # Read the keyboard.
                kbd_input = self.kbd.read()
                if kbd_input[0] == 1:
                    motor = 0.3
                elif kbd_input[2] == 1:
                    motor = -0.3
                if kbd_input[1] == 1:
                    servo_signal = -1
                elif kbd_input[3] == 1:
                    servo_signal = 1
                
                # Prevent the steering signal to be sent if currently writing in the terminal.
                if str(self.focus_get()) == ".terminal.input":
                    motor = 0
                    servo_signal = 0

                # Send the steering signals.
                msg_info = self.mqtt_client.publish("commands", "1 " + str(servo_signal) + " " + str(motor), qos=1)
                self.unacked_publish.add(msg_info.mid)
                msg_info.wait_for_publish()

            case "stop":
                # Stop the MQTT-loop if given the steering mode "stop".
                self.mqtt_client.loop_stop()
        self.after(100, self.steering)
        return
    
    # Create a graph from specified recorded data.
    def create_graph(self, file):
        self.graphs.append(graph.Graph(file))

    # Just wiggles the car a bit.
    def wiggle(self):
        msg_info = self.mqtt_client.publish("commands", "1 -1 0", qos=1)
        self.unacked_publish.add(msg_info.mid)
        msg_info.wait_for_publish()
        time.sleep(0.2)
        msg_info = self.mqtt_client.publish("commands", "1 1 0", qos=1)
        self.unacked_publish.add(msg_info.mid)
        msg_info.wait_for_publish()
        time.sleep(0.2)

    # Wiggles the car for specified time.
    def dance(self, times):
        for i in range(times):
            msg_info = self.mqtt_client.publish("commands", "1 -1 0", qos=1)
            self.unacked_publish.add(msg_info.mid)
            msg_info.wait_for_publish()
            time.sleep(0.2)
            msg_info = self.mqtt_client.publish("commands", "1 1 0", qos=1)
            self.unacked_publish.add(msg_info.mid)
            msg_info.wait_for_publish()
            time.sleep(0.2)

    # Just turn up the gas and drift bby!!!
    def drift(self):
        time.sleep(1)
        msg_info = self.mqtt_client.publish("commands", "1 -1 0.5", qos=1)
        self.unacked_publish.add(msg_info.mid)
        msg_info.wait_for_publish()




# Create an instance of GUI and start the loop.
UI = GUI("CarGoBrr2")
UI.mainloop()
