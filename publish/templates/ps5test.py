import time
from evdev import InputDevice, categorize, ecodes
gamepad = InputDevice('/dev/input/event13')

button_presses = {
    
}

absolutes = {
        0: 'L3 stick',      #left/right
        #1: 'L3 stick',      #up/down
        2: 'L2 trigger',
        5: 'R2 trigger',
}

CENTER = 0
BLIND = 0
steer = CENTER
gas = 0
triggers = [0, 0]

def update_gas(event):
    global triggers, gas
    if event.code == 2:             # Left trigger
        triggers[0] = value
    elif event.code == 5:           # Right trigger
        triggers[1] = value
    gas = 0.3 * (triggers[1]-triggers[0]) / 256


def update_steer(event):
    global steer
    if event.code == 0:
        steer = value / 128 - 1
    #elif event.code == 1:
        #left_stick[1] = value


if __name__ == '__main__':
    # print(gamepad)

    for event in gamepad.read_loop():
        # print(categorize(event))
        # print(event)

        if event.type == ecodes.EV_KEY and event.code in button_presses:       # any button press other than leftpad
            button, direction = button_presses[event.code], button_values[event.value]
            print(f'{button} {direction}') #ok

            if is_emergency(event):
                print('EMERGENCY BUTTON!')

        if event.type == ecodes.EV_ABS and event.code in absolutes:                     # leftpad, joystick motion, or L2/R2 triggers
            action, value = absolutes[event.code], event.value
            
            if event.code in [0, 1, 2, 5]:                                              # joystick motion
                if event.code in [0, 1]:                                                # left joystick moving
                    update_steer(event)
                elif event.code in [2, 5]:    #5, R2 moving
                    update_gas(event)         #update_right_joystick_position(event)

                if event.value > (CENTER - BLIND) and event.value < (CENTER + BLIND):   # skip printing the jittery center for the joysticks
                    continue
                        
                print(f'{steer} {gas} 0') #print(f'{left_joystick}, {right_joystick}')
                continue

            if event.code in [3, 4]:                                                    # L2/R2 triggers
                print(f'{action} {value}')
            elif event.code in [16, 17]:                                                # leftpad (d-pad) action
                action = decode_leftpad(event)
                print(action)

