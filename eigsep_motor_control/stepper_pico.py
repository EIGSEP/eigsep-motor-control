import time
from machine import Pin
import ujson

class Stepper:
    def __init__(self, pin_arr):
        """
        Class to initialize Pin pins on Raspberry Pi and control stepper
        motor rotations.

        Parameters
        -----------
        pin_arr: sequence of integers
            GPIO pin numbers following BCM ordering. Pins correspond to inputs
            on microstep driver in the following order: direction, pulse,
            clockwise direction, counterclockwise direction, enable.

        """
        self.cnt = 0
        self.SLEEP = 3
        self.direction_pin = pin_arr[0]  # 20
        self.pulse_pin = pin_arr[1]  # 21
        self.cw_direction = pin_arr[2]  # 0
        self.ccw_direction = pin_arr[3]  # 1
        self.enable_pin = pin_arr[4]  # 26

        self.cnt_arr = [0] * 1000
        self.cnt_ind = 0

        self.step_angle = 1.8  # our motor has a step angle of 1.8 degrees
        self.microstep = (
            4  # our motor driver has a microstep of 4, can be adjusted
        )
        self.full_box = 360  # Full rotation of the box
        self.gear_teeth = 113  # teeth amount on the driven gear

        self.direction_output = Pin(self.direction_pin, Pin.OUT)
        self.pulse_output = Pin(self.pulse_pin, Pin.OUT)
        self.enable_output = Pin(self.enable_pin, Pin.OUT)
        
        self.enable_output.value(1)

    def motor_calc(self, delay, rotation):
        """
        Calculates the speed of the motor and the amount of pulses needed for
        the platform to reach a given degree change. Also enables the motor.

        Parameters
        ----------
        delay: int
            The time between pulses in microseconds.
        rotation: int
            The degree change from the current position in degrees.
        """
        self.direction = rotation
        self.delay = delay
        self.pulse_amount = int(
            self.microstep * self.gear_teeth * self.direction / self.step_angle
        )
        self.box_max = int(
            self.microstep * self.gear_teeth * self.full_box / self.step_angle
        )
        #Pin.output(self.enable_pin, Pin.LOW)
        self.enable_output.value(0)

    def move(self, name):
        """
        Starts rotation of the platform and counts the amount of steps taken.

        Parameters
        ----------
        name: str
            The name of which motor is moving, either elevation or azimuth.
        """
        if self.direction > 0:
            self.direction_output.value(self.cw_direction)
            self.cnt += 1
        if self.direction < 0:
            self.direction_output.value(self.ccw_direction)
            self.cnt -= 1
        
        self.pulse_output.value(1)
        time.sleep_us(self.delay)
        self.pulse_output.value(0)
        time.sleep_us(self.delay)
        
        if self.cnt_ind < len(self.cnt_arr):
            self.cnt_arr[self.cnt_ind] = self.cnt
            self.cnt_ind += 1
            if self.cnt_ind >= len(self.cnt_arr):
                self.save()
                self.cnt_arr = [0] * len(self.cnt_arr)
                self.cnt_ind = 0

    def check(self, inf=False):
        """
        Checks if the amount of steps taken has reached the calculated amount,
        or if the platform has taken 2 full rotations in one direction. If it
        has taken 2 full rotations, the direction will reverse. If the number
        of steps is reached, the motors will disable and the function will
        return a 0.

        Parameters
        ----------
        None
        """

        if inf:
            if abs(self.cnt) >= abs(2 * self.box_max):
                self.direction *= -1
                time.sleep(self.SLEEP)
                print("reversing")
                return 0
        if not inf:
            if abs(self.cnt) >= abs(self.pulse_amount) or abs(self.cnt) >= 2*self.box_max:
                self.pulse_output.value(0)
                self.enable_output.value(1)
                print("disabled")
                return 0


    def save(self):
        fname = f'step_log_{time.ticks_ms()}.json'
        try:
            with open(fname, 'w') as f:
                ujson.dump(self.cnt_arr, f)
        except Exception as e:
            print(f'Error saving. {e}')


    def close(self):
        """
        Disables the motor

        Parameters
        ----------
        None
        """
        self.pulse_output.value(0)
        self.enable_output.value(1)

