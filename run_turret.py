from turret import Turret

MOTORS_REVERSED = (False, False)
MOTORS_RANGE = (300, 150)

TRIGGER_PIN = 22

MICRO_POS = None  # (X,Y)
MICRO_PINS = None   # (X,Y)

if __name__ == "__main__":
    t = Turret(friendly_mode=False,
        trigger_pin = TRIGGER_PIN,
        micro_pins = MICRO_PINS, micro_pos = MICRO_POS,
        motors_reversed = MOTORS_REVERSED, motor_range = MOTORS_RANGE,
        show_video=True)
    t.calibrate()
    t.motion_detection()