try:
    import cv2
except Exception as e:
    print("Warning: OpenCV not installed. To use motion detection, make sure you've properly configured OpenCV.")
import time
import _thread
import threading
import sys
import gpiozero
from time import sleep
import imutils
import RPi.GPIO as GPIO


### User Parameters ###

MOTOR_X_REVERSED = False
MOTOR_Y_REVERSED = False

MAX_STEPS_X = 19 #importante, debe coincidir con los if de los angulos
MAX_STEPS_Y = 4

RELAY_PIN = 23  # Adjusted for Blinka

#######################
# Setup for Adafruit Blinka
servoX =gpiozero.AngularServo(18,initial_angle=0, min_angle=0, max_angle=180, min_pulse_width=0.0005, max_pulse_width=0.0025)
servoY =gpiozero.AngularServo(24,initial_angle=80, min_angle=0, max_angle=180, min_pulse_width=0.0005, max_pulse_width=0.0025)

relay = gpiozero.OutputDevice(RELAY_PIN, active_high=False, initial_value=False)
#relay = digitalio.DigitalInOut(RELAY_PIN)
#relay.direction = digitalio.Direction.OUTPUT

class VideoUtils(object):
    """
    Helper functions for video utilities.
    """
    @staticmethod
    def live_video(camera_port=0):
        """
        Opens a window with live video.
        :param camera:
        :return:
        """

        video_capture = cv2.VideoCapture(camera_port)

        while True:
            # Capture frame-by-frame
            ret, frame = video_capture.read()

            # Display the resulting frame
            cv2.imshow('Video', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # When everything is done, release the capture
        video_capture.release()
        cv2.destroyAllWindows()

    @staticmethod
    def find_motion(callback, camera_port=0, show_video=False):

        camera = cv2.VideoCapture(camera_port)
        #time.sleep(0.25)

        # initialize the first frame in the video stream
        firstFrame = None
        tempFrame = None
        count = 0

        # loop over the frames of the video
        while True:
            # grab the current frame and initialize the occupied/unoccupied
            # text

            (grabbed, frame) = camera.read()

            # if the frame could not be grabbed, then we have reached the end
            # of the video
            if not grabbed:
                break

            # resize the frame, convert it to grayscale, and blur it
            frame = imutils.resize(frame, width=600)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (21, 21), 0)

            # if the first frame is None, initialize it
            if firstFrame is None:
                print("Waiting for video to adjust...")
                if tempFrame is None:
                    tempFrame = gray
                    continue
                else:
                    delta = cv2.absdiff(tempFrame, gray)
                    tempFrame = gray
                    tst = cv2.threshold(delta, 25, 255, cv2.THRESH_BINARY)[1]
                    #cv.adaptiveThreshold(img,255,cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY,11,2)
                    tst = cv2.dilate(tst, None, iterations=2)
                    if count > 30:
                        print("Done.\n Waiting for motion.")
                        if not cv2.countNonZero(tst) > 0:
                            firstFrame = gray
                        else:
                            continue
                    else:
                        count += 1
                        continue

            # compute the absolute difference between the current frame and
            # first frame
            frameDelta = cv2.absdiff(firstFrame, gray)
            thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]

            # dilate the thresholded image to fill in holes, then find contours
            # on thresholded image
            thresh = cv2.dilate(thresh, None, iterations=2)
            c = VideoUtils.get_best_contour(thresh.copy(), 2000)

            if c is not None:
                # compute the bounding box for the contour, draw it on the frame,
                # and update the text
                (x, y, w, h) = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                callback(c, frame)

            # show the frame and record if the user presses a key
            if show_video:
                #cv2.namedWindow("Security Feed", cv2.WINDOW_NORMAL)
                # FPS = 1/X
                # X = desired FPS
                FPS = 1/30
                FPS_MS = int(FPS * 1000)
                cv2.imshow("Security Feed", frame)
                cv2.imshow("TEst", thresh)
                #cv2.resizeWindow("Security Feed", 900, 600)
                key = cv2.waitKey(FPS_MS) & 0xFF

                # if the `q` key is pressed, break from the loop
                if key == ord("q"):
                    break

        # cleanup the camera and close any open windows
        camera.release()
        cv2.destroyAllWindows()

    @staticmethod
    def get_best_contour(imgmask, threshold):
        #im, contours, hierarchy = cv2.findContours(imgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours, hierarchy = cv2.findContours(imgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_area = threshold
        best_cnt = None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > best_area:
                best_area = area
                best_cnt = cnt
        return best_cnt


class Turret(object):
    """
    Class used for turret control.
    """
    def __init__(self, friendly_mode=True):
        self.friendly_mode = friendly_mode
        #
        # Stepper motor 1
        #Usar esto angulos para el arranque del servoX
        self.current_x_steps = 0
        self.servoX = servoX                
        servoX.angle = 45
        sleep(1)
        servoX.angle = 90
        sleep(1)
        servoX.angle = 135
        sleep(1)
        servoX.angle = 90
        sleep(1)
        
        # Stepper motor 2
        #Usar esto angulos para el arranque del servoY
        self.current_y_steps = 0
        self.servoY = servoY                
        servoY.angle = 60
        sleep(1)
        servoY.angle = 80
        sleep(1)
        servoY.angle = 100
        sleep(1)
        servoY.angle = 80
        sleep(1)
        
        
        
        # Relay
        #GPIO.setmode(GPIO.BCM)
        #GPIO.setup(RELAY_PIN, GPIO.OUT)
        #GPIO.output(RELAY_PIN, GPIO.LOW)

        
    def motion_detection(self, show_video=False):
        """
        Uses the camera to move the turret. OpenCV ust be configured to use this.
        :return:
        """
        VideoUtils.find_motion(self.__move_axis, show_video=show_video)

    def __move_axis(self, contour, frame):
        (v_h, v_w) = frame.shape[:2]
        (x, y, w, h) = cv2.boundingRect(contour)
        print("x: %s" % (str(x)))
        print("y: %s" % (str(y)))
        print("w: %s" % (str(w)))
        print("h: %s" % (str(h)))
        
        
        # find height
        target_steps_x = round((MAX_STEPS_X * ((x + (w / 2)) / v_w)),0)
        target_steps_y = round((2*MAX_STEPS_Y*(y+h/2) / v_h) - MAX_STEPS_Y,0)
        print("current x: %s" % (str(self.current_x_steps)))
        print("x-step: %s, y-step: %s" % (str(target_steps_x), str(target_steps_y)))
        #print("current x: %s, current y: %s" % (str(self.current_x_steps), str(self.current_y_steps)))
        

        t_x = threading.Thread()
        #t_y = threading.Thread()
        t_fire = threading.Thread()

        # # # move x
        
        if (target_steps_x) == 19:
            self.current_x_steps = target_steps_x
            t_x = threading.Thread(target=Turret.moveTurret,args=(self.servoX, 45,))
        elif (target_steps_x) == 18:
            self.current_x_steps = target_steps_x
            t_x = threading.Thread(target=Turret.moveTurret,args=(self.servoX, 50,))
        elif (target_steps_x) == 17:
            self.current_x_steps = target_steps_x
            t_x = threading.Thread(target=Turret.moveTurret,args=(self.servoX, 55,))
        elif (target_steps_x) == 16:
            self.current_x_steps = target_steps_x
            t_x = threading.Thread(target=Turret.moveTurret,args=(self.servoX, 60,))
        elif (target_steps_x) == 15:
            self.current_x_steps = target_steps_x
            t_x = threading.Thread(target=Turret.moveTurret,args=(self.servoX, 65,))
        elif (target_steps_x) == 14:
            self.current_x_steps = target_steps_x
            t_x = threading.Thread(target=Turret.moveTurret,args=(self.servoX, 70,))
        elif (target_steps_x) == 13:
            self.current_x_steps = target_steps_x
            t_x = threading.Thread(target=Turret.moveTurret,args=(self.servoX, 75,))
        elif (target_steps_x) == 12:
            self.current_x_steps = target_steps_x
            t_x = threading.Thread(target=Turret.moveTurret,args=(self.servoX, 80,))
        elif (target_steps_x) == 11:
            self.current_x_steps = target_steps_x
            t_x = threading.Thread(target=Turret.moveTurret,args=(self.servoX, 85,))
        elif (target_steps_x) == 10:
            self.current_x_steps = target_steps_x
            t_x = threading.Thread(target=Turret.moveTurret,args=(self.servoX, 90,))
        elif (target_steps_x) == 9:
            self.current_x_steps = target_steps_x
            t_x = threading.Thread(target=Turret.moveTurret,args=(self.servoX, 95,))
        elif (target_steps_x) == 8:
            self.current_x_steps = target_steps_x
            t_x = threading.Thread(target=Turret.moveTurret,args=(self.servoX, 100,))
        elif (target_steps_x) == 7:
            self.current_x_steps = target_steps_x
            t_x = threading.Thread(target=Turret.moveTurret,args=(self.servoX, 105,))
        elif (target_steps_x) == 6:
            self.current_x_steps = target_steps_x
            t_x = threading.Thread(target=Turret.moveTurret,args=(self.servoX, 110,))
        elif (target_steps_x) == 5:
            self.current_x_steps = target_steps_x
            t_x = threading.Thread(target=Turret.moveTurret,args=(self.servoX, 115,))
        elif (target_steps_x) == 4:
            self.current_x_steps = target_steps_x
            t_x = threading.Thread(target=Turret.moveTurret,args=(self.servoX, 120,))
        elif (target_steps_x) == 3:
            self.current_x_steps = target_steps_x
            t_x = threading.Thread(target=Turret.moveTurret,args=(self.servoX, 125,))
        elif (target_steps_x) == 2:
            self.current_x_steps = target_steps_x
            t_x = threading.Thread(target=Turret.moveTurret,args=(self.servoX, 130,))
        elif (target_steps_x) == 1:
            self.current_x_steps = target_steps_x
            t_x = threading.Thread(target=Turret.moveTurret,args=(self.servoX, 135,))
        
        
        # # move y
        # if (target_steps_y - self.current_y_steps) > 0:
        #     self.current_y_steps += 1
        #     if MOTOR_Y_REVERSED:
        #         t_y = threading.Thread(target=Turret.move_backward, args=(self.sm_y, 2,))
        #     else:
        #         t_y = threading.Thread(target=Turret.move_forward, args=(self.sm_y, 2,))
        # elif (target_steps_y - self.current_y_steps) < 0:
        #     self.current_y_steps -= 1
        #     if MOTOR_Y_REVERSED:
        #         t_y = threading.Thread(target=Turret.move_forward, args=(self.sm_y, 2,))
        #     else:
        #         t_y = threading.Thread(target=Turret.move_backward, args=(self.sm_y, 2,))
        #
        # # fire if necessary
        if not self.friendly_mode:
            t_fire = threading.Thread(target=Turret.fire)
            #if abs(target_steps_y - self.current_y_steps) <= 2 and abs(target_steps_x - self.current_x_steps) <= 2:
            #t_fire = threading.Thread(target=Turret.fire)

        t_x.start()
        #t_y.start()
        t_fire.start()

        t_x.join()
        #t_y.join()
        t_fire.join()

    
    @staticmethod
    def fire():
        #GPIO.output(RELAY_PIN, GPIO.HIGH)
        #GPIO.output(RELAY_PIN, True)
        relay.on()
        print("fire")
        time.sleep(0.1)
        relay.off()
        #GPIO.output(RELAY_PIN, False)
        #GPIO.output(RELAY_PIN, GPIO.LOW)
        
    def moveTurret(servo, angle):
        """
        Moves the stepper motor forward the specified number of steps.
        :param motor:
        :param steps:
        :return:
        """
        servo.angle = angle
        sleep(0.1)
        print("angle: %s" % (str(angle)))

    
if __name__ == "__main__":
    t = Turret(friendly_mode=False)
    #t.calibrate()
    #if raw_input("Live video? (y, n)\n").lower() == "y":
    
    #Descomentar
    if input("Live video? (y, n)\n").lower() == "y":
        t.motion_detection(show_video=True)
    else:
        t.motion_detection()    


