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
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.on_subscribe = self.on_subscribe
        self.mqtt_client.on_unsubscribe = self.on_unsubscribe
        self.mqtt_client.on_publish = self.on_publish
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
    
    def on_subscribe(self, client, userdata, mid, reason_code_list, properties):
        # Since we subscribed only for a single channel, reason_code_list contains
        # a single entry
        if reason_code_list[0].is_failure:
            print(f"Broker rejected your subscription: {reason_code_list[0]}")
        else:
            print(f"Broker granted the following QoS: {reason_code_list[0].value}")

    def on_unsubscribe(self, client, userdata, mid, reason_code_list, properties):
        # Be careful, the reason_code_list is only present in MQTTv5.
        # In MQTTv3 it will always be empty
        if len(reason_code_list) == 0 or not reason_code_list[0].is_failure:
            print("unsubscribe succeeded (if SUBACK is received in MQTTv3 it success)")
        else:
            print(f"Broker replied with failure: {reason_code_list[0]}")
        client.disconnect()

    def on_message(self, client, userdata, message):
        # userdata is the structure we choose to provide, here it's a list()
        if message.topic == "commands":
            information.data_list.append(message.payload)
            information.data_list.pop(0)

        elif message.topic == "data":
            print(message.payload)

        elif message.topic == "cones":
            message.payload = message.payload.decode("utf-8")
            if message.payload == "":
                return
            information.cones.clear()
            message.payload = message.payload.rstrip(";")
            cones = message.payload.split(";")
            for cone in cones:
                cone_info_list = cone.split(",")
                cone_tuple = (float(cone_info_list[1])/10,
                            float(cone_info_list[0])/10,
                            float(cone_info_list[2])/10)
                information.cones.append(cone_tuple)

        elif message.topic == "bezier":
            message.payload = message.payload.decode("utf-8")
            if message.payload == "":
                return
            information.bezier[0].clear()
            #print("Punkter: "+ message.payload)
            message.payload = message.payload.rstrip(";")
            def_points = message.payload.split(";")
            for def_point in def_points:
                def_point_info_list = def_point.split(",")
                def_point_tuple = (float(def_point_info_list[1])/10,
                                float(def_point_info_list[0])/10)
                information.bezier[0].append(def_point_tuple)

        elif message.topic == "curve":
            message.payload = message.payload.decode("utf-8")
            if message.payload == "":
                return
            information.bezier[1].clear()
            #print("Kurva: "+ message.payload)
            message.payload = message.payload.rstrip(";")
            def_waypoints = message.payload.split(";")
            for def_waypoint in def_waypoints:
                def_waypoint_info_list = def_waypoint.split(",")
                def_waypoint_tuple = (float(def_waypoint_info_list[1])/10,
                                    float(def_waypoint_info_list[0])/10)
                information.bezier[1].append(def_waypoint_tuple)

        # We only want to process 10 messages
        if len(userdata) >= 10:
            #client.unsubscribe("hi")
            return

    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            print(f"Failed to connect: {reason_code}. loop_forever() will retry connection")
        else:
            # we should always subscribe from on_connect callback to be sure
            # our subscribed is persisted across reconnections.
            client.subscribe([("commands",0), ("data", 0), ("cones", 0), ("curve", 0), ("bezier", 0)])

    def on_publish(self, client, userdata, mid, reason_code, properties):
        # reason_code and properties will only be present in MQTTv5. It's always unset in MQTTv3
        try:
            userdata.remove(mid)
        except KeyError:
            print("on_publish() is called with a mid not present in unacked_publish")
            print("This is due to an unavoidable race-condition:")
            print("* publish() return the mid of the message sent.")
            print("* mid from publish() is added to unacked_publish by the main thread")
            print("* on_publish() is called by the loop_start thread")
            print("While unlikely (because on_publish() will be called after a network round-trip),")
            print(" this is a race-condition that COULD happen")
            print("")
            print("The best solution to avoid race-condition is using the msg_info from publish()")
            print("We could also try using a list of acknowledged mid rather than removing from pending list,")
            print("but remember that mid could be re-used !")

UI = GUI("CarGoBrr2")
UI.mainloop()
