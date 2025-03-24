from time import sleep
import time
import RPi.GPIO as gpio
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
import threading

parser = ArgumentParser(description = 'Motor velocity', formatter_class = ArgumentDefaultsHelpFormatter)

parser.add_argument('-t', dest = 'time', type = int, default=225, help = 'Delay between pulses sent to stepper motor in microseconds')
parser.add_argument('-de', dest = 'degree_e', type = int, default=0, help = 'Number of degrees to rotate the box, (0, 360)')

parser.add_argument('-da', dest = 'degree_a', type = int, default=0, help = 'Number of degrees to rotate the antenna, (0, 360)')

args = parser.parse_args()
delay = int(args.time)
box_angle = args.degree_e # theta?
antenna_angle = args.degree_a # phi?

print(f'Angular Velocity of {1.8/delay} radians/sec')

class Stepper:
    def __init__(self, pin_arr):
        """
        Class to initialize gpio pins on Raspberry Pi and control stepper motor rotations.

        Parameteres
        -----------
        pin_arr: sequence of integers
            GPIO pin numbers following BCM ordering. Pins correspond to inputs on microstep driver in the following order: direction, pulse, clockwise direction, counterclockwise direction, enable.

        """
        self.cnt = 0
        self.save_cnt = 0
        self.direction_pin   = pin_arr[0]# 20
        self.pulse_pin       = pin_arr[1]# 21
        self.cw_direction    = pin_arr[2]# 0 
        self.ccw_direction   = pin_arr[3]# 1
        self.enable_pin = pin_arr[4]#26
        self.step_angle = 1.8 # our motor has a step angle of 1.8 degrees
        self.full_shaft = 360 # Full rotation of motor shaft
        self.full_box = 360 # Full rotation of the box
        self.worm = 7 # 7? teeth amount on the worm gear
        self.gear = 113 # teeth amount on the driven gear
        self.full_rotation = 113/7 # 1 rod rotation=all 7 teeth cycled?
        gpio.cleanup()
        gpio.setmode(gpio.BCM)
        gpio.setup(self.direction_pin, gpio.OUT)
        gpio.setup(self.pulse_pin, gpio.OUT)
        gpio.setup(self.enable_pin, gpio.OUT)

   
    def motor_calc(self, delay, rotation):
        """
        Calculates the speed of the motor and the amount of pulses needed for the platform to reach a given degree change. Also enables the motor.

        Parameters
        ----------
        delay: int
            The time between pulses in microseconds.
        rotation: int
            The degree change from the current position in degrees.
        """
        self.direction = rotation
        self.delay = abs(delay/1e6)
        self.inc = self.worm/self.full_shaft
        self.pulse_amount = int(self.worm*self.direction*20/(self.full_rotation*self.inc*self.step_angle)) # not sure why the 20 works, i think it takes 20 rotations of the motor for the entire worms teeth to complete a cycle
        self.box_max = int(self.worm*360*20/(self.full_rotation*self.inc*self.step_angle)) 
        gpio.output(self.enable_pin, gpio.LOW)

    def move(self):
        """
        Starts rotation of the platform and counts the amount of steps taken.

        Parameters
        ----------
        None
        """ 
        if self.direction > 0:
            gpio.output(self.direction_pin, self.cw_direction)
            print('positive')
        if self.direction < 0:
            gpio.output(self.direction_pin, self.ccw_direction)
            print('negative')
        gpio.output(self.pulse_pin, gpio.HIGH)
        sleep(self.delay)
        gpio.output(self.pulse_pin, gpio.LOW)
        sleep(self.delay)
        self.cnt += 1
        print(self.cnt, self.pulse_amount)
        print(self.pulse_amount*self.step_angle/self.full_rotation)
    

    def check(self):
        """
        Checks if the amount of steps taken has reached the calculated amount, or if the platform has taken 2 full rotations in one direction. If its taken 2 full rotations, the direction will reverse. 
        
        Parameters
        ----------
        None
        """
        if abs(self.cnt) >= 2*self.box_max:
            self.direction *= -1
            time.sleep(3)
        if self.cnt >= abs(self.pulse_amount):
            gpio.output(self.enable_pin, gpio.HIGH)
            print('disabled')
            return 0


    def close(self):
        """
        Disables the motor

        Parameters
        ----------
        None
        """
        gpio.output(self.enable_pin, gpio.HIGH)


elevation_pins = [20,21,0,1,26] 
azimuth_pins = [17,22,0,1,27]

def main_function(pins, d, angle):
    """
    Function that initializes the motor, calculates speed and direction, and moves the motor until the degree change is reached.

    Parameters
    ---------- 
    pins: sequence of integers
        GPIO pin numbers following BCM ordering. Pins correspond to inputs on microstep driver in the following order: direction, pulse, clockwise direction, counterclockwise direction, enable.
     d: int
        The time between pulses in microseconds.
    angle: int
        The degree change from the current position in degrees.
    """
    try:
        motor = Stepper(pins)
        motor.motor_calc(d, angle)
        while True:
            motor.move()
            if motor.check() == 0:
                break
    
    except KeyboardInterrupt:
        motor.close()
    
def both():
    """
    Rotates both motors simultaneously

    Parameters
    ----------
    None
    """
    t1 = threading.Thread(target=main_function, args=(elevation_pins, delay, box_angle))
    t2 = threading.Thread(target=main_function, args=(azimuth_pins, delay, antenna_angle))
    t1.start()
    t2.start()
    t1.join()
    t2.join()

if box_angle and antenna_angle != 0: # Rotates both motors
    both()

if antenna_angle != 0 and box_angle == 0: # Rotates antenna
    main_function(azimuth_pins, delay, antenna_angle)

if box_angle != 0 and antenna_angle == 0: # Rotates platform
    main_function(elevation_pins, delay, box_angle)

