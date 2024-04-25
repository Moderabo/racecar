import tkinter as tk
import os
import time
import path_overview
import terminal
import data_overview
import mqtt
import rorry
import xbox
import read_keyboard
import paho.mqtt.client as client
import information

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
        #============================

        #=====Set up MQTT client=====
        self.unacked_publish = set()
        self.mqtt_client = client.Client(client.CallbackAPIVersion.VERSION2)
        self.mqtt_client.on_connect = mqtt.on_connect
        self.mqtt_client.on_message = mqtt.on_message
        self.mqtt_client.on_subscribe = mqtt.on_subscribe
        self.mqtt_client.on_unsubscribe = mqtt.on_unsubscribe
        self.mqtt_client.on_publish = mqtt.on_publish
        self.mqtt_client.user_data_set(self.unacked_publish)
        #=============================

        try:
            self.controller = xbox.XboxController()
        except:
            print("No controller detected!")
        
        self.steering_mode = "stop"

        self.update_GUI()
        self.is_on = True
        self.steering()

        self.kbd = read_keyboard.Keyboard()

        self.bind("<Button-1>", self.click_event)

    def __del__(self):
        os.system(f'''cmd /c "netsh wlan connect name=eduroam"''')
        return

    def setup_mqtt_protocol(self):
        os.system(f'''cmd /c "netsh wlan connect name=CarGoBRR2"''')
        time.sleep(5)
        self.mqtt_client.connect("10.42.0.1")

    def click_event(self, event):
        event.widget.focus_set()

    def change_driving_mode(self, mode):
        if self.steering_mode == "stop":
            self.mqtt_client.loop_start()
        msg_info = self.mqtt_client.publish("commands", "0 0 0", qos=1)
        self.unacked_publish.add(msg_info.mid)
        msg_info.wait_for_publish()
        self.steering_mode = mode
        return

    def update_GUI(self): 
        self.data_widget.update()
        self.path_widget.update()
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

                msg_info = self.mqtt_client.publish("commands", str(servo_signal) + " "+ str(motor) + " " + "0", qos=1)
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

                msg_info = self.mqtt_client.publish("commands", str(servo_signal) + " "+ str(motor) + " " + "0", qos=1)
                self.unacked_publish.add(msg_info.mid)

                msg_info.wait_for_publish()

            case "stop":
                self.mqtt_client.loop_stop()
            
        #print(len(information.cones))
        self.after(1000, self.steering)
        return

UI = GUI("CarGoBrr2")
UI.mainloop()
