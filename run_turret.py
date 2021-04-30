from turret import Turret

MOTORS_REVERSED = (False, False)
MOTOR_RANGE = (300, 150)

TRIGGER_PIN = 22

MICRO_POS = None  # (X,Y)
MICRO_PINS = None   # (X,Y)

if __name__ == "__main__":
    t = Turret(friendly_mode=False,
        trigger_pin = TRIGGER_PIN,
        micro_pins = MICRO_PINS, micro_pos = MICRO_POS,
        motors_reversed = MOTOR_REVERSED, motor_range = MOTOR_RANGE)
    t.calibrate()
    t.motion_detection(show_video=True)