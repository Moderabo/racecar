from inputs import get_gamepad
import math
import threading

import paho.mqtt.client as mqtt

class XboxController(object):
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):

        self.LeftJoystickY = 0.0
        self.LeftJoystickX = 0.0
        self.RightJoystickY = 0.0
        self.RightJoystickX = 0.0
        self.LeftTrigger = 0.0
        self.RightTrigger = 0.0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightThumb = 0
        self.Back = 0
        self.Start = 0
        self.LeftDPad = 0
        self.RightDPad = 0
        self.UpDPad = 0
        self.DownDPad = 0

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()


    def read(self): # return the buttons/triggers that you care about in this methode
        x_cord = self.LeftJoystickX
        y_cord = self.LeftJoystickY
        rb = self.RightBumper
        lb = self.LeftBumper
        rt = self.RightTrigger
        lt = self.LeftTrigger
        return [x_cord, y_cord, rb, lb, rt, lt]


    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.code == 'ABS_Y':
                    self.LeftJoystickY = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_X':
                    self.LeftJoystickX = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RY':
                    self.RightJoystickY = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RX':
                    self.RightJoystickX = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_Z':
                    self.LeftTrigger = event.state / XboxController.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'ABS_RZ':
                    self.RightTrigger = event.state / XboxController.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'BTN_TL':
                    self.LeftBumper = event.state
                elif event.code == 'BTN_TR':
                    self.RightBumper = event.state
                elif event.code == 'BTN_SOUTH':
                    self.A = event.state
                elif event.code == 'BTN_NORTH':
                    self.Y = event.state #previously switched with X
                elif event.code == 'BTN_WEST':
                    self.X = event.state #previously switched with Y
                elif event.code == 'BTN_EAST':
                    self.B = event.state
                elif event.code == 'BTN_THUMBL':
                    self.LeftThumb = event.state
                elif event.code == 'BTN_THUMBR':
                    self.RightThumb = event.state
                elif event.code == 'BTN_SELECT':
                    self.Back = event.state
                elif event.code == 'BTN_START':
                    self.Start = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY1':
                    self.LeftDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY2':
                    self.RightDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY3':
                    self.UpDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY4':
                    self.DownDPad = event.state

def clamp(minimum, x, maximum):
    return max(minimum, min(x, maximum))

def on_publish(client, userdata, mid, reason_code, properties):
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

if __name__ == '__main__':
    controller = XboxController()
    is_on = True
    end_it = 0

    unacked_publish = set()
    mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    mqttc.on_publish = on_publish

    mqttc.user_data_set(unacked_publish)
    mqttc.connect("10.42.0.1")
    mqttc.loop_start()

    while is_on:
        controller_inputs = controller.read()
        x_cord = controller_inputs[0]
        rt = controller_inputs[4]
        lt = controller_inputs[5]

        servo_signal = x_cord

        motor = rt * 0.5 - lt * 0.5

        if controller_inputs[2] != 0 and controller_inputs[3] != 0:
            is_on = False
            end_it = 1

        if -0.1 <= abs(servo_signal) <= 0.1:
            servo_signal = 0.0
        else:
            servo_signal = clamp(-1, servo_signal, 1)

        motor = clamp(-1.0, motor, 1.0)

        print(str(servo_signal) + " "+ str(motor) + " " + str(end_it))

        # Our application produce some messages
        msg_info = mqttc.publish("commands", str(servo_signal) + " "+ str(motor) + " " + str(end_it), qos=1)
        unacked_publish.add(msg_info.mid)

        # Wait for all message to be published
        # while len(unacked_publish):
        #     time.sleep(0.001)

        # Due to race-condition described above, the following way to wait for all publish is safer
        msg_info.wait_for_publish()

        if controller_inputs[2] != 0 and controller_inputs[3] != 0:
            is_on = False

    mqttc.disconnect()
    mqttc.loop_stop()