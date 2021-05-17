import atexit
import time
import threading

class StepperBase(object):
    def __init__(self, name, reverse = False):
        self.thread_start = False
        self.pos = 0
        self.target = 0
        self.end = 0
        self.name = name
        atexit.register(self.__end)
        self.forward_dir = -1 if reverse else 1

    def start_loop(self):
        self.flag = threading.Event()
        self.thread = threading.Thread(target=self.__loop, daemon=True)
        self.thread.start()
        self.thread_started = True

    def __get_flag(self):
        return self.flag

    def __get_thread(self):
        return self.thread

    def set_target(self, target):
        self.target = int(target)
        self.__get_flag.set()
        
    def on_target(self):
        return abs(self.target - self.pos) < 2
    
    def __update():
        if (abs(self.target - self.pos) >= 1):
            self.step(self.forward_dir if self.target - self.pos > 0 else -self.forward_dir)
            return True
        else:
            return False

    def __loop(self):
        while not self.end:
            if not self.__update():
                self.flag.wait()
        
    def __end(self):
        self.end = True
        if self.thread_started:
            self.__get_flag().set()
            self.__get_thread().join()

class StepperReal(StepperBase):
    def __init__(self, name, reverse):
        StepperBase.__init__(self, name, reverse=reverse)
        
    def calibrate(self, micro_pin, micro_pos):
        return threading.Thread(self.__calibrate_run, args=(micro_pin, micro_pos))
        
    def __calibrate_run(self, micro_pin, micro_pos):
        GPIO.setup(micro_pin, GPIO.IN)
        while not GPIO.input(micro_pin):
            self.step(-1)
        self.pos = micro_pos
