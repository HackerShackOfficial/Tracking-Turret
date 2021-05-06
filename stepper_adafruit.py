import RPi.GPIO as GPIO
import adafruit_motorkit
from adafruit_motor import stepper
from stepper_base import StepperReal

# Adafruit Motor Hat
class Stepper(StepperReal):
    def __init__(self, kit, is_stepper2, reverse, name):
        StepperReal.__init__(self, name, reverse)
        self.motor = kit.stepper2 if is_stepper2 else kit.stepper1
                
    def step(self, steps):
        self.pos += steps
        direction = stepper.FORWARD if steps > 0 else stepper.BACKWARD
        for i in range(abs(steps)):
            self.motor.onestep(direction=direction, style=stepper.DOUBLE)
