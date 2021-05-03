try:
    import adafruit_motorkit
    import RPi.GPIO as GPIO
    from stepper import Stepper
    from gun import Gun
    has_motors = True
except:
    has_motors = False
    print("***************************************************************")
    print("Failed to initialise Motors or GPIO.  Using dummy")
from motion_sensor import MotionSensor
import atexit
import dummy

class Turret(object):
    def __init__(self, motors_reversed, motor_range,
            friendly_mode=True,
            trigger_pin = None,
            micro_pins = None, micro_pos = None,
            show_video = False):
        global has_motors
        
        self.friendly_mode = friendly_mode
        self.micro_pins = micro_pins
        self.micro_pos = micro_pos
        self.motor_range = motor_range

        # create a default object, no changes to I2C address or frequency
        if has_motors:
            try:
                mh = adafruit_motorkit.MotorKit()
                GPIO.setmode(GPIO.BCM)
                self.stepper_x = Stepper(mh, False, motors_reversed[0], "X")
                self.stepper_y = Stepper(mh, True, motors_reversed[1], "Y")
            except ValueError:
                has_motors = False
                
        if not has_motors:
            self.stepper_x = dummy.StepperMotor("X")
            self.stepper_y = dummy.StepperMotor("Y")
            
        atexit.register(self.__turn_off_motors)

        if has_motors:
            self.gun = Gun(trigger_pin, self.stepper_x, self.stepper_y, friendly_mode)
        else:
            self.gun = dummy.Gun(self.stepper_x, self.stepper_y, friendly_mode)
        self.motion_sensor = MotionSensor(self.__on_motion, self.__on_no_motion, show_video=show_video)

    def calibrate(self):
        if self.micro_pins is None or self.micro_pos is None:
            print("No Microswitch pin specified.  Skipping calibration.")
        else:
            print("Calibrating...")
            t_x = self.stepper_x.calibrate(self.micro_pins[0], self.micro_pos[0])
            t_y = self.stepper_y.calibrate(self.micro_pins[1], self.micro_pos[1])
            t_x.join()
            t_y.join()

    def motion_detection(self):
        self.stepper_x.start_loop()
        self.stepper_y.start_loop()
        self.gun.start_loop()
        
        self.motion_sensor.find_motion()

    def __on_motion(self, motion_center, frame):
        target_steps_x = self.motor_range[0] * motion_center[0]
        target_steps_y = self.motor_range[1] * motion_center[1]

        print ("x: %s, y: %s" % (str(target_steps_x), str(target_steps_y)))
        print ("current x: %s, current y: %s" % (str(self.stepper_x.pos), str(self.stepper_y.pos)))

        self.stepper_x.set_target(target_steps_x)
        self.stepper_y.set_target(target_steps_y)
        self.gun.set_fire_on_target(True)
        
    def __on_no_motion(self, frame):
        self.stepper_x.set_target(0)
        self.stepper_y.set_target(0)
        self.gun.set_fire_on_target(False)

    def __turn_off_motors(self):
        # TODO: FIX THIS
        '''
        self.mh.getMotor(1).run(MotorKit.RELEASE)
        self.mh.getMotor(2).run(MotorKit.RELEASE)
        self.mh.getMotor(3).run(MotorKit.RELEASE)
        self.mh.getMotor(4).run(MotorKit.RELEASE)
        '''
        pass
