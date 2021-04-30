import RPi.GPIO as GPIO
import time
import threading
import atexit

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
        atexit.register(self.__end)
        
    def start_loop(self):
        self.thread = threading.Thread(target=self.__loop, daemon=True)
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