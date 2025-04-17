import time
import RPi.GPIO as gpio


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
        self.SLEEP = 3
        self.direction_pin   = pin_arr[0]# 20
        self.pulse_pin       = pin_arr[1]# 21
        self.cw_direction    = pin_arr[2]# 0 
        self.ccw_direction   = pin_arr[3]# 1
        self.enable_pin = pin_arr[4]#26
        
        self.step_angle = 1.8 # our motor has a step angle of 1.8 degrees
        self.microstep = 4 # our motor driver has a microstep of 4, can be adjusted
        self.full_box = 360 # Full rotation of the box
        self.gear_teeth = 113 # teeth amount on the driven gear
        
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
        self.pulse_amount = int(self.microstep*self.gear_teeth*self.direction/self.step_angle)
        self.box_max = int(self.microstep*self.gear_teeth*self.full_box/self.step_angle)
        gpio.output(self.enable_pin, gpio.LOW)

    def move(self, name):
        """
        Starts rotation of the platform and counts the amount of steps taken.

        Parameters
        ----------
        name: str
            The name of which motor is moving, either elevation or azimuth.
        """ 
        if self.direction > 0:
            gpio.output(self.direction_pin, self.cw_direction)
            self.cnt += 1
            print('positive')
        if self.direction < 0:
            gpio.output(self.direction_pin, self.ccw_direction)
            self.cnt -= 1
            print('negative')
        gpio.output(self.pulse_pin, gpio.HIGH)
        time.sleep(self.delay)
        gpio.output(self.pulse_pin, gpio.LOW)
        time.sleep(self.delay)
        #self.cnt += 1
        print(f'{str(name)} step count: {self.cnt}, max step: {self.pulse_amount}')
    

    def check(self):
        """
        Checks if the amount of steps taken has reached the calculated amount, or if the platform has taken 2 full rotations in one direction. If its taken 2 full rotations, the direction will reverse. If the number of steps is reached, the motors will disable and the function will return a 0 
        
        Parameters
        ----------
        None
        """
        if abs(self.cnt) >= abs(2*self.box_max):
            self.direction *= -1
            time.sleep(self.SLEEP)
            print('reversing')
            return 0
        if abs(self.cnt) >= abs(self.pulse_amount):
            gpio.output(self.pulse_pin, gpio.LOW)
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
        gpio.output(self.pulse_pin, gpio.LOW)
        gpio.output(self.enable_pin, gpio.HIGH)

