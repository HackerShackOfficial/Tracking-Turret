import RPi.GPIO as GPIO
import adafruit_motorkit
from adafruit_motor import stepper
from stepper_base import StepperReal
import threading
import time
import config

# Adafruit Motor Hat
class Stepper(StepperReal):
    def __init__(self, kit, is_stepper2, reverse, name):
        StepperReal.__init__(self, name, reverse)
        self.kit = kit
        self.motor = kit.stepper2 if is_stepper2 else kit.stepper1
                
    def step(self, steps):
        self.pos += steps
        direction = stepper.FORWARD if (steps > 0) != self.reverse else stepper.BACKWARD
        for i in range(abs(steps)):
            self.motor.onestep(direction=direction, style=stepper.DOUBLE)

    threads = {}
        # Dictionary with key = kit, values are
        # Tuples (thread, flag, list of Steppers)

    def start_loop(self):
        if self.kit not in Stepper.threads:
            Stepper.end = False
            flag = threading.Event()
            steppers = [self]
            thread = threading.Thread(target=Stepper.__loop, daemon=True, args=(flag, steppers))
            Stepper.threads[self.kit] = (thread, flag, steppers)
            thread.start()
        else:
            Stepper.threads[self.kit][2].append(self)

    @staticmethod
    def __loop(flag, steppers):
        while not Stepper.end:
            action = False
            for s in steppers:
                if s.update():
                    action = True
            if action:
                if config.STEPPER_DELAY:
                    time.sleep(config.STEPPER_DELAY)
            else:
                flag.wait() # No action wait for event flag (new target)

    def _get_flag(self):
        return Stepper.threads[self.kit][1]

    def _get_thread(self):
        return Stepper.threads[self.kit][0]