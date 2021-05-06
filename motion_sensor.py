import cv2
import threading
import time
import imutils

class FrameGrabException(Exception):
    pass

class MotionSensor(object):
    INITIALISED = 0
    GETTING_EMPTY_FRAME = 1
    GOT_EMPTY_FRAME = 2
    TRACKING = 3

    def __init__(self, callback_motion=None, callback_nomotion=None, camera_port=0, show_video=False, max_frame_rate = 0.250):
        self.camera = cv2.VideoCapture(camera_port)
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # Reduce Lag
        self.end = False
        self.show_video = show_video
        self.callback_motion = callback_motion
        self.callback_nomotion = callback_nomotion

        self.max_frame_rate = max_frame_rate
        self.last_frame_time = time.time()

        self.last_image = None

        self.state = MotionSensor.INITIALISED
        self.static_count = 0
        self.center_norm = (0,0)

        self.image_width = 500
        self.blur_radius = 21
        self.threshold = 25
        self.static_count_limit_start = 20
        self.static_count_limit_live = 40

    def quit(self):
        self.end = True

    @property
    def state_string(self):
        s = ("Initialised", "Getting Empty Frame", "Got Empty Frame", "Tracking")
        return s[self.state]

    def grab_image(self):
        now = time.time()
        if self.max_frame_rate - (now - self.last_frame_time) > 0:
            time.sleep(self.max_frame_rate - (now - self.last_frame_time))
        self.last_frame_time = time.time()

        grabbed, frame = self.camera.read()
        if not grabbed:
            raise FrameGrabException()

        # resize the frame, convert it to grayscale, and blur it
        frame = imutils.resize(frame, width=self.image_width)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (self.blur_radius, self.blur_radius), 0)

        return frame, gray
    
    def compare(self, base, current):
        delta = cv2.absdiff(base, current)
        tst = cv2.threshold(delta, self.threshold, 255, cv2.THRESH_BINARY)[1]
        return cv2.dilate(tst, None, iterations=2)

    # Get initial "empty" image.
    # Wait until there are 20 similar images with 250ms sleep between them.
    # Similar is defined as having zero thresholded delta from
    # first image of the set (canidate), after greying, blurring.
    def get_empty_frame(self):
        self.state = MotionSensor.GETTING_EMPTY_FRAME
        frame, candidate = self.grab_image()
        self.static_count = 0
        while self.static_count < self.static_count_limit_start and not self.end:
            frame, gray = self.grab_image()
            if gray.shape != candidate.shape:   # Image width has changed restart
                candidate = gray
                self.static_count = 0
            diff = self.compare(candidate, gray)

            if self.show_video:
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
                self.static_count += 1
            else:
                self.static_count = 0   # Motion detected, try current image as base
                candidate = gray

            cv2.putText(frame, str(self.static_count), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (240, 240, 240), thickness=2, lineType=cv2.LINE_AA)
            cv2.putText(frame, str(self.static_count), (10, frame.shape[0]-30), cv2.FONT_HERSHEY_SIMPLEX, 1, (30, 30, 30), thickness=2, lineType=cv2.LINE_AA)
            self.callback_nomotion(frame)
            self.last_image = frame
            if self.show_video:
                cv2.imshow("Frame", frame)
            time.sleep(0.250)
        
        cv2.destroyAllWindows()
        self.state = MotionSensor.GOT_EMPTY_FRAME

        return candidate
    
    def find_motion(self):
        try:
            base = self.get_empty_frame()
            recent = base
            self.state = MotionSensor.TRACKING
            self.static_count = 0

            # loop over the frames of the video
            while not self.end:
                # Find contour in difference between base and current
                frame, gray = self.grab_image()
                if gray.shape != base.shape:    # Image width has changed.  Get new base.
                    base = self.get_empty_frame()
                    recent = base

                diff = self.compare(base, gray)
                c = MotionSensor.get_best_contour(diff.copy(), 5000)

                if c is None:
                    self.callback_nomotion(frame)
                else:
                    (x, y, w, h) = cv2.boundingRect(c)
                    center = (x+int(w/2), y+int(h/2))
                    self.center_norm = (2*(center[0]/frame.shape[1] - 0.5), 2*(center[1]/frame.shape[0] - 0.5))  # Range -1 to 1
                    
                    # Draw bounds and contour on frame
                    cv2.drawContours(frame, c, -1, (10, 10, 10), 1)
                    cv2.circle(frame, center, 15, (240, 240, 240), 1)
                    cv2.line(frame, (center[0]-24, center[1]), (center[0]+24, center[1]), (240, 240, 240), 1)
                    cv2.line(frame, (center[0], center[1]-24), (center[0], center[1]+24), (240, 240, 240), 1)
                    self.callback_motion(self.center_norm, frame)

                # show the frame and record if the user presses a key
                self.last_image = frame
                if self.show_video:
                    cv2.imshow("Feed", frame)
                    key = cv2.waitKey(1) & 0xFF

                    # if the `q` key is pressed, break from the lop
                    if key == ord("q"):
                        break
                    
                # check for recent motion
                diff = self.compare(recent, gray)
                diff_count = cv2.countNonZero(diff)
                if diff_count == 0:  # No motion
                    self.static_count += 1
                    if  self.static_count > self.static_count_limit_live: # No motion for 40 frames
                        # Set recent as new base
                        base = recent
                        recent = gray
                        self.static_count = 0
                        print("New base set")
                else:
                    self.static_count = 0   # Motion detected, try set recent to current
                    recent = gray


        finally:
            # cleanup the camera and close any open windows
            self.camera.release()
            cv2.destroyAllWindows()

    @staticmethod
    def get_best_contour(imgmask, threshold):
        contours, _ = cv2.findContours(imgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_area = threshold
        best_cnt = None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > best_area:
                best_area = area
                best_cnt = cnt
        return best_cnt
