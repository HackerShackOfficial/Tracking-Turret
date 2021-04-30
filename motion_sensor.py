import cv2
import threading
import time
import imutils

class FrameGrabException(Exception):
    pass

class MotionSensor(object):
    def __init__(self, camera_port=0, diag=False):
        self.camera = cv2.VideoCapture(camera_port)
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # Reduce Lag
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
        
        cv2.destroyAllWindows()

        return candidate
    
    def find_motion(self, callback_motion, callback_nomotion, show_video=False):
        try:
            base = self.get_empty_frame()
            recent = base
            static_count = 0

            # loop over the frames of the video
            while True:
                # Find contour in difference between base and current
                frame, gray = self.grab_image()
                diff = self.compare(base, gray)
                c = MotionSensor.get_best_contour(diff.copy(), 5000)

                if c is None:
                    callback_nomotion(frame)
                else:
                    (x, y, w, h) = cv2.boundingRect(c)
                    center = (x+int(w/2), y+int(h/2))
                    center_norm = (2*(center[0]/frame.shape[1] - 0.5), 2*(center[1]/frame.shape[0] - 0.5))  # Range -1 to 1
                    if show_video:
                        print(center)
                        # Draw bounds and contour on frame
                        cv2.drawContours(frame, c, -1, (0, 255, 255), 1)
                        cv2.circle(frame, center, 15, (0, 0, 255), 1)
                        cv2.line(frame, (center[0]-24, center[1]), (center[0]+24, center[1]), (0, 0, 255), 1)
                        cv2.line(frame, (center[0], center[1]-24), (center[0], center[1]+24), (0, 0, 255), 1)
                    callback_motion(center_norm, frame)

                # show the frame and record if the user presses a key
                if show_video:
                    cv2.imshow("Feed", frame)
                    key = cv2.waitKey(1) & 0xFF

                    # if the `q` key is pressed, break from the lop
                    if key == ord("q"):
                        break
                    
                # check for recent motion
                diff = self.compare(recent, gray)
                diff_count = cv2.countNonZero(diff)
                if diff_count == 0:  # No motion
                    static_count += 1
                    if static_count > 40: # No motion for 40 frames
                        # Set recent as new base
                        base = recent
                        recent = gray
                        static_count = 0
                        print("New base set")
                else:
                    static_count = 0   # Motion detected, try set recent to current
                    recent = gray


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
