from stepper_base import StepperBase
import atexit
import time
import threading

class StepperMotor(StepperBase):
    def __init__(self, name):
        StepperBase.__init__(self, name)
                
    def step(self, steps):
        self.pos += steps
        time.sleep(0.1 * abs(steps))

    def calibrate(self, micro_pin, micro_pos):
        pass

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
    
    @property
    def on_target(self):
        return self.x.on_target() and self.y.on_target()

    def step(self, steps):
        self.pos += steps
        time.sleep(0.1 * abs(steps))

    def __loop(self):
        while not self.end:
            self.firing = not self.friendly and self.fire_on_target and self.on_target()
            time.sleep(1)
            
    def __end(self):
        self.end = True
        if self.thread_started:
            self.thread.join()