from pynput import keyboard

# Class to read the keyboard input.
class Keyboard():

    def __init__(self):

        self.listener = keyboard.Listener(
        on_press=self.on_press,
        on_release=self.on_release)
        self.listener.start()

        self.A = 0
        self.W = 0
        self.S = 0
        self.D = 0

    def on_press(self, key):
        try:
            if key.char == 'a':
                self.A = 1
            if key.char == 's':
                self.S = 1
            if key.char == 'w':
                self.W = 1
            if key.char == 'd':
                self.D = 1
        except:
            return

    def on_release(self, key):
        try:
            if key.char == 'a':
                self.A = 0
            if key.char == 's':
                self.S = 0
            if key.char == 'w':
                self.W = 0
            if key.char == 'd':
                self.D = 0
        except:
            return

    def read(self):
        return [self.W, self.A, self.S, self.D]


