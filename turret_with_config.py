from turret import Turret

MOTORS_REVERSED = (False, False)
MOTORS_RANGE = (300, 150)

TRIGGER_PIN = 22

MICRO_POS = None  # (X,Y)
MICRO_PINS = None   # (X,Y)

ADA_FRUIT_STEPPERS = False
DUMMY = True
WAVESHARE_STEPPERS = False

if WAVESHARE_STEPPERS:
    X_DIR_PIN  =    13
    X_STEP_PIN =    19
    X_ENABLE_PING = 12
    X_MODE_PINS =   (16, 17, 20)
    Y_DIR_PIN =     24
    Y_STEP_PIN =    18
    Y_ENABLE_PIN =  4
    Y_MODE_PINS =   (21, 22, 27)

def create():
    if ADA_FRUIT_STEPPERS:
        import adafruit_motorkit
        import stepper_adafruit
        import RPi.GPIO as GPIO
        mh = adafruit_motorkit.MotorKit()
        GPIO.setmode(GPIO.BCM)
        steppers = (stepper_adafruit.Stepper(mh, False, MOTORS_REVERSED[0], "X"),
                    stepper_adafruit.Stepper(mh, True, MOTORS_REVERSED[1], "Y"))
    elif DUMMY:
        import dummy
        steppers = (dummy.StepperMotor("X"), dummy.StepperMotor("Y"))
    elif WAVESHARE_STEPPERS:
        import stepper_waveshare
        steppers = (stepper_waveshare.Stepper(X_DIR_PIN, X_STEP_PIN, X_ENABLE_PIN, X_MODE_PINS, MOTORS_REVERSED[0], "X"),
                    stepper_waveshare.Stepper(Y_DIR_PIN, Y_STEP_PIN, Y_ENABLE_PIN, Y_MODE_PINS, MOTORS_REVERSED[1], "Y"))

    turret = Turret(steppers = steppers,
        friendly_mode=False,
        trigger_pin = TRIGGER_PIN,
        micro_pins = MICRO_PINS, micro_pos = MICRO_POS,
        motor_range = MOTORS_RANGE,
        show_video=True)
    return turret

def start(turret):
    turret.calibrate()
    turret.motion_detection()

def go():
    turret = create()
    start(turret)
