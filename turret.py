import cv2
import time
import threading
import atexit
import sys
import termios
import contextlib

import imutils
import RPi.GPIO as GPIO
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor

### User Parameters ###

MOTOR_X_REVERSED = False
MOTOR_Y_REVERSED = False

MAX_STEPS_X = 30
MAX_STEPS_Y = 15

RELAY_PIN = 22

MICRO_X_POS    = -35
MICRO_Y_POS = -20
MICRO_X_PIN = 0000000000000000000000 ######### TODO
MICRO_Y_PIN = 0000000000000000000000 ######### TODO

#######################

class FrameGrabException(Exception):
    pass

class MotionSensor(object):
    def __init__(self, camera_port=0, diag=False):
        self.camera = cv2.VideoCapture(camera_port)
        self.diag = diag

    def grab_image(self):
        grabbed, frame = self.camera.read()
        if not grabbed:
            raise FrameGrabException()

        # resize the frame, convert it to grayscale, and blur it
        frame = imutils.resize(frame, width=500)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)
        return frame, gray
    
    def compare(self, base, current):
        delta = cv2.absdiff(base, current)
        tst = cv2.threshold(delta, 25, 255, cv2.THRESH_BINARY)[1]
        return cv2.dilate(tst, None, iterations=2)

    # Get initial "empty" image.
    # Wait until there are 20 similar images with 250ms sleep between them.
    # Similar is defined as having zero thresholded delta from
    # first image of the set (canidate), after greying, blurring.
    def get_empty_frame(self):
        frame, candidate = self.grab_image()
        static_count = 0
        while static_count < 20:
            frame, gray = self.grab_image()
            diff = self.compare(candidate, gray)

            if self.diag:
                #cv2.imshow("Frame", frame)
                #cv2.imshow("Base", candidate)
                #cv2.imshow("Current", gray)
                #cv2.imshow("Delta", delta)
                cv2.imshow("Threshold and Dilate", diff)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    return None

            diff_count = cv2.countNonZero(diff)
            if diff_count == 0:  # No motion
                static_count += 1
            else:
                static_count = 0   # Motion detected, try current image as base
                candidate = gray
            print(static_count, "similar images.", diff_count)
            time.sleep(0.250)

        return candidate
    
    def find_motion(self, callback_motion, callback_nomotion, show_video=False):
        try:
            base = self.get_empty_frame()

            # loop over the frames of the video
            while True:
                frame, gray = self.grab_image()
                diff = self.compare(base, gray)
                c = MotionSensor.get_best_contour(diff.copy(), 5000)

                if c is None:
                    callback_nomotion(frame)
                else:
                    # compute the bounding box for the contour, draw it on the frame,
                    # and update the text
                    (x, y, w, h) = cv2.boundingRect(c)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    callback_motion(c, frame)

                # show the frame and record if the user presses a key
                if show_video:
                    cv2.imshow("Feed", frame)
                    key = cv2.waitKey(1) & 0xFF

                    # if the `q` key is pressed, break from the lop
                    if key == ord("q"):
                        break

        finally:
            # cleanup the camera and close any open windows
            self.camera.release()
            cv2.destroyAllWindows()

    @staticmethod
    def get_best_contour(imgmask, threshold):
        contours, hierarchy = cv2.findContours(imgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_area = threshold
        best_cnt = None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > best_area:
                best_area = area
                best_cnt = cnt
        return best_cnt

class Stepper(object):
    def __init__(self, hat, i_motor, reversed):
        self.motor = hat.getStepper(200, i_motor)
        self.motor.setSpeed(5)
        self.pos = 0
        self.reversed = reversed
        self.target = 0
        self.flag = threading.Event()
        self.end = False
        self.thread = threading.Thread(self.__loop)
        atexit.register(self.__end)
        
    def start_loop(self):
        self.thread.start()
        
    def set_target(self, target):
        self.target = target
        self.flag.set()
        
    def on_target(self):
        return abs(self.target - self.pos) < 2
        
    def __loop(self):
        while not self.end:
            if (self.target == self.pos):
                # If at target pause thread for target change
                self.flag.wait()
            else:
                self.step(2 if self.target - self.pos < 0 else -2)
                
    def __step(self, steps):
        self.pos += steps
        if self.reversed:
            steps = -steps
        self.motor.step(abs(steps), Adafruit_MotorHAT.FORWARD if steps > 0 else Adafruit_MotorHAT.BACKWARD, Adafruit_MotorHAT.INTERLEAVE)
        
    def calibrate(self, micro_pin, micro_pos):
        return threading.Thread(self.__calibrate_run, args=(micro_pin, micro_pos))
        
    def __calibrate_run(self, micro_pin, micro_pos):
        GPIO.setup(micro_pin, GPIO.IN)
        while not GPIO.input(micro_pin):
            self.__step(-1)
        self.pos = micro_pos
        
    def __end(self):
        self.end = True
        self.flag.set()
        self.thread.join()

class Gun(object):
    def __init__(self, relay, stepper_x, stepper_y, friendly):
        self.x = stepper_x
        self.y = stepper_y
        self.relay = relay
        self.friendly = friendly
        self.fire_on_target = False
        GPIO.setup(relay, GPIO.OUT)
        GPIO.output(relay, GPIO.LOW)
        self.end = False
        self.thread = threading.Thread(self.__loop)
        atexit.register(self.__end)
        
    def start_loop(self):
        self.thread.start()
    
    def set_friendly(self, friendly):
        self.friendly = friendly
        
    def set_fire_on_target(self, fire_on_target):
        self.fire_on_target = fire_on_target
    
    def __loop(self):
        while not self.end:
            fire = not self.friendly and self.fire_on_target and self.x.on_target() and self.y.on_target()
            GPIO.output(self.relay, GPIO.HIGH if fire else GPIO.LOW)
            time.sleep(1)
            
    def __end(self):
        self.end = True
        self.thread.join()
        GPIO.output(self.relay, GPIO.LOW)

class Turret(object):
    def __init__(self, friendly_mode=True):
        self.friendly_mode = friendly_mode

        # create a default object, no changes to I2C address or frequency
        self.mh = Adafruit_MotorHAT()
        atexit.register(self.__turn_off_motors)

        GPIO.setmode(GPIO.BCM)
        self.stepper_x = Stepper(self.mh, 1, MOTOR_X_REVERSED)
        self.stepper_y = Stepper(self.mh, 2, MOTOR_Y_REVERSED)
        self.gun = Gun(RELAY_PIN, self.stepper_x, self.stepper_y, friendly_mode)
        self.motion_sensor = MotionSensor()

    def calibrate(self):
        print("Calibrating...")
        t_x = self.stepper_x.calibrate(MICRO_X_PIN, MICRO_X_POS)
        t_y = self.stepper_y.calibrate(MICRO_Y_PIN, MICRO_Y_POS)
        t_x.join()
        t_y.join()

    def motion_detection(self, show_video=False):
        """
        Uses the camera to move the turret. OpenCV ust be configured to use this.
        :return:
        """
        self.stepper_x.start_loop()
        self.stepper_y.start_loop()
        self.gun.start_loop()
        
        self.motion_sensor.find_motion(self.__on_motion, self.__on_no_motion, show_video=show_video)

    def __on_motion(self, contour, frame):
        (v_h, v_w) = frame.shape[:2]
        (x, y, w, h) = cv2.boundingRect(contour)

        # find height
        target_steps_x = (2*MAX_STEPS_X * (x + w / 2) / v_w) - MAX_STEPS_X
        target_steps_y = (2*MAX_STEPS_Y * (y + h / 2) / v_h) - MAX_STEPS_Y

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
        """
        Recommended for auto-disabling motors on shutdown!
        :return:
        """
        self.mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

if __name__ == "__main__":
    t = Turret(friendly_mode=False)
    t.calibrate()
    t.motion_detection(show_video=False)
