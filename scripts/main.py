from stepper_pico import Stepper
import _thread
import sys
import ujson
import time




elevation_pins = [11, 13, 0, 1, 9]
azimuth_pins = [17, 22, 0, 1, 27]

print('connected')

def wait_for_command():
    sys.stdout.write("waiting")
#    sys.stdout.flush()
    line = sys.stdin.readline()
    try:
        args = ujson.loads(line)
    except Exception as e:
        print("bad json:", e)
        return None
    return args


def main_function(pins, d, angle, name, inf):
    """
    Function that initializes the motor, calculates speed and direction, and
    moves the motor until the degree change is reached.

    Parameters
    ----------
    pins: sequence of integers
        GPIO pin numbers following BCM ordering. Pins correspond to inputs on
        microstep driver in the following order: direction, pulse, clockwise
        direction, counterclockwise direction, enable.
     d: int
        The time between pulses in microseconds.
    angle: int
        The degree change from the current position in degrees.
    name: str
        The name of the motor to rotate, either elevation or azimuth
    inf: bool
        Rotate stepper motor indefinitely
    """
    try:
        motor = Stepper(pins)
        motor.motor_calc(d, angle)
        if inf:
            motor.move(name)
            while True:
                motor.move(name)
                motor.check(inf)
        else:
            while True:
                motor.move(name)
                if motor.check(inf) == 0:
                    break

    except KeyboardInterrupt:
        motor.close()



def run(box_angle, antenna_angle, delay, inf):
    if antenna_angle != 0:
        main_function(azimuth_pins, delay, antenna_angle, "azimuth", inf)
        print('first elif')
    elif box_angle != 0:
        main_function(elevation_pins, delay, box_angle, "elevation", inf)
        print('second elif')
    else:
        sys.stdout.write('No angle given')

if __name__ == "__main__":
    while True:
        pkt = wait_for_command()
        if pkt is None:
            time.sleep(1)
            continue
        delay, box_angle, antenna_angle, inf = pkt
        inf = bool(inf)
        print(pkt)
        if box_angle == 0 and antenna_angle == 0 and not inf:
            print('Exiting')
            break
        run(box_angle, antenna_angle, delay, inf)
