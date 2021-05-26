from DRV8825 import DRV8825
from stepper_base import StepperReal

# Adafruit Motor Hat
class Stepper(StepperReal):
    def __init__(self, dir_pin, step_pin, enable_pin, mode_pins, reverse, name):
        StepperReal.__init__(self, name, reverse)
        self.motor = DRV8825(dir_pin, step_pin, enable_pin, mode_pins)
        self.motor.set_micro_step(DRV8825.STEP_ONE)

        self.current_direction = -1
                
    def step(self, steps):
        self.pos += steps
        direction = DRV8825.FORWARD if (steps > 0) != self.reverse else DRV8825.BACKWARD
        if direction != self.current_direction:
            self.motor.set_direction(direction)
            self.current_direction = direction
        for _ in range(abs(steps)):
            self.motor.step()
