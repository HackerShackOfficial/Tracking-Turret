import RPi.GPIO as GPIO
import adafruit_motorkit
from adafruit_motor import stepper
import atexit
import threading

class Stepper(object):
    def __init__(self, kit, is_stepper2, reverse, name):
        self.thread_start = False
        self.name = name
        self.motor = kit.stepper2 if is_stepper2 else kit.stepper1
        self.pos = 0
        self.reverse = reverse
        self.target = 0
        self.end = False
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
                
    def step(self, steps):
        self.pos += steps
        print((self.name, self.pos, self.target))
        direction = stepper.FORWARD if (steps > 0) != self.reverse else stepper.BACKWARD
        for i in range(abs(steps)):
            self.motor.onestep(direction=direction, style=stepper.DOUBLE)
        
    def calibrate(self, micro_pin, micro_pos):
        return threading.Thread(self.__calibrate_run, args=(micro_pin, micro_pos))
        
    def __calibrate_run(self, micro_pin, micro_pos):
        GPIO.setup(micro_pin, GPIO.IN)
        while not GPIO.input(micro_pin):
            self.step(-1)
        self.pos = micro_pos
        
    def __end(self):
        self.end = True
        if self.thread_started:
            self.flag.set()
            self.thread.join()
