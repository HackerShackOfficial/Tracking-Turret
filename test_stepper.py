import adafruit_motorkit
import time
import turret

mh = adafruit_motorkit.MotorKit()
stepper = turret.Stepper(mh, False, False)

for i in range(30):
    stepper.step(i)
    time.sleep(0.25)
    stepper.step(-i)
    time.sleep(0.25)