from turret import VideoUtils
import datetime

def on_motion(contour, frame):
    print(datetime.now, "MOTION")
    
def on_no_motion(frame):
    print(datetime.now, "NO MOTION")
    
VideoUtils.find_motion(on_motion, on_no_motion, show_video=True)