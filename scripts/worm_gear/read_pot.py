import time
from eigsep_motor_control import Potentiometer

pot = Potentiometer()

while True:
    v = pot.read_volts()
    print(f"az: {v[0]:.3f}, alt: {v[1]:.3f}")
    time.sleep(0.5)
