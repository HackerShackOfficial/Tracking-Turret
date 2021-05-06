import RPi.GPIO as GPIO
import time

MotorDir = [
    'forward',
    'backward',
]

ControlMode = [
    'hardward',
    'softward',
]

class DRV8825():
    def __init__(self, dir_pin, step_pin, enable_pin, mode_pins):
        self.dir_pin = dir_pin
        self.step_pin = step_pin        
        self.enable_pin = enable_pin
        self.mode_pins = mode_pins
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.enable_pin, GPIO.OUT)
        GPIO.setup(self.mode_pins, GPIO.OUT)
        
    def digital_write(self, pin, value):
        GPIO.output(pin, value)
        
    def Stop(self):
        self.digital_write(self.enable_pin, 1)
    
    def SetMicroStep(self, mode, stepformat):
        """
        (1) mode
            'hardward' :    Use the switch on the module to control the microstep
            'software' :    Use software to control microstep pin levels
                Need to put the All switch to 0
        (2) stepformat
            ('fullstep', 'halfstep', '1/4step', '1/8step', '1/16step', '1/32step')
        """
        microstep = ('fullstep',
                     'halfstep',
                     '1/4step',
                     '1/8step',
                     '1/16step',
                     '1/32step')
        step = microstep.index[stepformat]

        print( "Control mode:",mode)
        if (mode == ControlMode[1]):
            print( "set pins")
            self.set_micro_step(step)

    STEP_ONE = 0
    STEP_HALF = 1
    STEP_QUARTER = 2
    STEP_EIGHTH = 3
    STEP_SIXTEENTH = 4
    STEP_1_32 = 5
    def set_micro_step(self, step):
        microstep = ((0, 0, 0),
                     (1, 0, 0),
                     (0, 1, 0),
                     (1, 1, 0),
                     (0, 0, 1),
                     (1, 0, 1))
        self.digital_write(self.mode_pins, microstep[step])
        
    def TurnStep(self, Dir, steps, stepdelay=0.005):
        if (Dir == MotorDir[0]):
            print( "forward")
            self.set_direction(0)
        elif (Dir == MotorDir[1]):
            print( "backward")
            self.set_direction(1)
        else:
            print( "the dir must be : 'forward' or 'backward'")
            self.digital_write(self.enable_pin, 1)
            return

        if (steps == 0):
            return
            
        print( "turn step:",steps)
        for i in range(steps):
            this.step(stepdelay)

    FORWARD = 0
    BACKWARD = 1

    def set_direction(self, dir):
        self.digital_write(self.enable_pin, 0)
        self.digital_write(self.dir_pin, dir)

    def step(self, stepdelay=0.005):
        self.digital_write(self.step_pin, True)
        time.sleep(stepdelay)
        self.digital_write(self.step_pin, False)
        time.sleep(stepdelay)
