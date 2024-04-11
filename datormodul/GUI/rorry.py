import paho.mqtt.subscribe as sub
import time

class Rorry():

    def __init__(self):
        sub.callback(self.on_message_print, "hi", hostname="10.42.0.1", userdata={"message_count": 0})
        return
    
    def on_message_print(self, client, userdata, message):
        print("%s %s" % (message.topic, message.payload))
        userdata["message_count"] += 1
        print(userdata["message_count"])

bil = Rorry()