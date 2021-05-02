import time
import threading
import atexit

class StepperMotor(object):
    def __init__(self, name):
        self.thread_start = False
        self.pos = 0
        self.target = 0
        self.end = 0
        self.name = name
        atexit.register(self.__end)

    def start_loop(self):
        self.flag = threading.Event()
        self.thread = threading.Thread(target=self.__loop, daemon=True)
        self.thread.start()
        self.thread_started = True

    def set_target(self, target):
        self.target = int(target)
        self.flag.set()
        
    def on_target(self):
        return abs(self.target - self.pos) < 2
        
    def __loop(self):
        while not self.end:
            if (self.target == self.pos):
                # If at target pause thread for target change
                self.flag.wait()
            else:
                self.step(1 if self.target - self.pos > 0 else -1)
            time.sleep(0.1)
                
    def step(self, steps):
        self.pos += steps
        print((self.name, self.pos, self.target))

    def calibrate(self, micro_pin, micro_pos):
        pass
        
    def __end(self):
        self.end = True
        if self.thread_started:
            self.flag.set()
            self.thread.join()

class Gun(object):
    def start_loop(self):
        pass

    def set_fire_on_target(self, x):
        pass