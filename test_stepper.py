import adafruit_motorkit
import time

mh = adafruit_motorkit.MotorKit()
stepper = Stepper(mh, 1, MOTOR_X_REVERSED)

for i in range(30):
    stepper.step(i)
    time.sleep(0.25)
    stepper.step(-i)
    time.sleep(0.25)