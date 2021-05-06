try:
    import adafruit_motorkit
    import RPi.GPIO as GPIO
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
    def __init__(self, motor_range,
            steppers,
            friendly_mode=True,
            trigger_pin = None,
            micro_pins = None, micro_pos = None,
            show_video = False):
        global has_motors
        
        self.friendly_mode = friendly_mode
        self.micro_pins = micro_pins
        self.micro_pos = micro_pos
        self.motor_range = motor_range

        self.stepper_x = steppers[0]
        self.stepper_y = steppers[1]

        self.override_motion = False
            
        atexit.register(self.__turn_off_motors)

        if has_motors:
            self.gun = Gun(trigger_pin, self.stepper_x, self.stepper_y, friendly_mode)
        else:
            self.gun = dummy.Gun(self.stepper_x, self.stepper_y, friendly_mode)
        self.motion_sensor = MotionSensor(self.__on_motion, self.__on_no_motion, show_video=show_video)

    @property
    def uses_dummy(self):
        return not has_motors

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
        if not self.override_motion:
            target_steps_x = self.motor_range[0] * motion_center[0]
            target_steps_y = self.motor_range[1] * motion_center[1]

            self.stepper_x.set_target(target_steps_x)
            self.stepper_y.set_target(target_steps_y)
            self.gun.set_fire_on_target(True)

    def __on_no_motion(self, frame):
        if not self.override_motion:
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

    def override_target(self, x, y):
        self.override_motion = True
        self.stepper_x.set_target(x * self.motor_range[0])
        self.stepper_y.set_target(y * self.motor_range[1])