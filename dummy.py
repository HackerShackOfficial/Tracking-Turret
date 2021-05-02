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
            if (abs(self.target - self.pos) >= 1):
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
    def __init__(self, stepper_x, stepper_y, friendly):
        self.thread_started = False
        self.x = stepper_x
        self.y = stepper_y
        self.friendly = friendly
        self.fire_on_target = False
        self.end = False
        atexit.register(self.__end)

        self.firing = False
        
    def start_loop(self):
        self.thread = threading.Thread(target=self.__loop, daemon=True)
        self.thread.start()
        self.thread_started = True
    
    def set_friendly(self, friendly):
        self.friendly = friendly
        
    def set_fire_on_target(self, fire_on_target):
        self.fire_on_target = fire_on_target
    
    def on_target(self):
        return self.x.on_target() and self.y.on_target()

    def __loop(self):
        while not self.end:
            self.firing = not self.friendly and self.fire_on_target and self.on_target()
            time.sleep(1)
            
    def __end(self):
        self.end = True
        if self.thread_started:
            self.thread.join()