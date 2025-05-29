from eigsep_motor_control import Stepper
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
import threading


parser = ArgumentParser(
    description="Motor velocity", formatter_class=ArgumentDefaultsHelpFormatter
)

parser.add_argument(
    "-t",
    dest="time",
    type=int,
    default=225,
    help="Delay between pulses sent to stepper motor in microseconds",
)
parser.add_argument(
    "-de",
    dest="degree_e",
    type=int,
    default=0,
    help="Number of degrees to rotate the box, (0, 360)",
)

parser.add_argument(
    "-da",
    dest="degree_a",
    type=int,
    default=0,
    help="Number of degrees to rotate the antenna, (0, 360)",
)

parser.add_argument(
    "-i",
    dest="inf_",
    action="store_true",
    help="Setting to rotate stepper motor infinitely",
)

args = parser.parse_args()
delay = int(args.time)
box_angle = args.degree_e  # theta?
antenna_angle = args.degree_a  # phi?
inf = args.inf_

print(f"Angular Velocity of {1.8 / delay} radians/sec")


elevation_pins = [20, 21, 0, 1, 26]
azimuth_pins = [17, 22, 0, 1, 27]


def main_function(pins, d, angle, name, inf=False):
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
                motor.check()
        else:
            while True:
                motor.move(name)
                if motor.check() == 0:
                    break

    except KeyboardInterrupt:
        motor.close()


def both(name1, name2):
    """
    Rotates both motors simultaneously

    Parameters
    ----------
    None
    """
    t1 = threading.Thread(
        target=main_function,
        args=(elevation_pins, delay, box_angle, name1, inf),
    )
    t2 = threading.Thread(
        target=main_function,
        args=(azimuth_pins, delay, antenna_angle, name2, inf),
    )
    t1.start()
    t2.start()
    t1.join()
    t2.join()


if box_angle != 0 and antenna_angle != 0:  # Rotates both motors
    box_name = "elevation"
    antenna_name = "azimuth"
    both(box_name, antenna_name)

if antenna_angle != 0 and box_angle == 0:  # Rotates antenna
    antenna_name = "azimuth"
    main_function(azimuth_pins, delay, antenna_angle, antenna_name, inf)

if box_angle != 0 and antenna_angle == 0:  # Rotates platform
    box_name = "elevation"
    main_function(elevation_pins, delay, box_angle, box_name, inf)
