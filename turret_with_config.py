try:
    from config import *
except:
    import sys
    print("*************************************************************************************************************")
    print("Could not import config.  Copy config.py.removeextension to config.py and enter the correct hardware options.")
    print("*************************************************************************************************************")
    sys.exit()
from turret import Turret

def create():
    if STEPPER_CONFIG == ADA_FRUIT_STEPPERS:
        import adafruit_motorkit
        import stepper_adafruit
        import RPi.GPIO as GPIO
        mh = adafruit_motorkit.MotorKit()
        GPIO.setmode(GPIO.BCM)
        steppers = (stepper_adafruit.Stepper(mh, False, MOTORS_REVERSED[0], "X"),
                    stepper_adafruit.Stepper(mh, True, MOTORS_REVERSED[1], "Y"))
    elif STEPPER_CONFIG == DUMMY_STEPPERS:
        import dummy
        steppers = (dummy.StepperMotor("X"), dummy.StepperMotor("Y"))
    elif STEPPER_CONFIG == WAVESHARE_STEPPERS:
        import stepper_waveshare
        steppers = (stepper_waveshare.Stepper(X_DIR_PIN, X_STEP_PIN, X_ENABLE_PIN, X_MODE_PINS, MOTORS_REVERSED[0], "X"),
                    stepper_waveshare.Stepper(Y_DIR_PIN, Y_STEP_PIN, Y_ENABLE_PIN, Y_MODE_PINS, MOTORS_REVERSED[1], "Y"))

    turret = Turret(steppers = steppers,
        friendly_mode=False,
        trigger_pin = TRIGGER_PIN,
        micro_pins = MICRO_PINS, micro_pos = MICRO_POS,
        motor_range = MOTORS_RANGE,
        show_video = SHOW_VIDEO)
    return turret

def start(turret):
    turret.calibrate()
    turret.motion_detection()

def go():
    turret = create()
    start(turret)
