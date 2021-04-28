from turret import MotionSensor
from datetime import datetime

def on_motion(center, frame):
    print(datetime.now(), "MOTION", center)
    
def on_no_motion(frame):
    print(datetime.now(), "NO MOTION")
    
motion_sensor = MotionSensor(diag=True)

motion_sensor.find_motion(on_motion, on_no_motion, show_video=True)