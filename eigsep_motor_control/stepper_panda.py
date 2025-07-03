import numpy as np
import serial
import json

class Stepper:
    """
    Stepper provides an interface to control a stepper motor via a microcontroller
    over a serial connection. It supports calculating steps, sending azimuth/elevation
    movement commands, reading position updates, and sending a stop command.

    Attributes:
        microstep (int): Microstepping factor (default 1).
        gear_teeth (int): Number of gear teeth (default 113).
        step_angle (float): Step angle in degrees (default 1.8).
        ser (serial.Serial): Serial connection object.
    """
    def __init__(self, pico):
        """
        Initialize the Stepper object and open the serial connection.

        Args:
            pico (str): The serial port identifier for the Pico (e.g., '/dev/ttyACM0').
        """
        self.microstep = 1
        self.gear_teeth = 113
        self.step_angle = 1.8
        self.ser = serial.Serial(
            port=pico,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )

    def calc_step(self, deg):
        """
        Calculate the number of steps required for a given degree of movement.

        Args:
            deg (float): Angle in degrees to rotate.

        Returns:
            int: Number of steps needed to rotate the specified angle.
        """
        return int(round(self.microstep * self.gear_teeth * abs(deg) / self.step_angle))

    def calc_deg(self, step):
        """
        Calculate the angle in degrees corresponding to a given number of steps.

        Args:
            step (float): The number of motor steps.

        Returns:
            float: The angle in degrees that corresponds to the given number of steps.
        """
        return step * self.step_angle / (self.microstep * self.gear_teeth)

    def move_el(self, deg, delay=225, pulses_az=0, dir_az=1, rep=100):
        """
        Send a command to move the stepper motor in elevation.

        Args:
            deg (float): Degrees to move in elevation (positive or negative).
            delay (int, optional): Delay between pulses (default 225).
            pulses_az (int, optional): Azimuth pulses (default 0).
            dir_az (int, optional): Azimuth direction (default 1).
            rep (int, optional): Report frequency or repetition count (default 100).
        """
        pulses = self.calc_step(deg)
        dir_el = 1 if deg > 0 else -1

        cmd = {
            "delay": delay,
            "pulses_az": pulses_az,
            "dir_az": dir_az,
            "pulses_el": pulses,
            "dir_el": dir_el,
            "report": rep
        }
        line = json.dumps(cmd) + "\n"
        self.ser.write(line.encode('utf-8'))

    def move_az(self, deg, delay=225, pulses_el=0, dir_el=1, rep=100):
        """
        Send a command to move the stepper motor in azimuth.

        Args:
            deg (float): Degrees to move in azimuth (positive or negative).
            delay (int, optional): Delay between pulses (default 225).
            pulses_el (int, optional): Elevation pulses (default 0).
            dir_el (int, optional): Elevation direction (default 1).
            rep (int, optional): Report frequency or repetition count (default 100).
        """
        pulses = self.calc_step(deg)
        dir_az = 1 if deg > 0 else -1

        cmd = {
            "delay": delay,
            "pulses_az": pulses,
            "dir_az": dir_az,
            "pulses_el": pulses_el,
            "dir_el": dir_el,
            "report": rep
        }
        line = json.dumps(cmd) + "\n"
        self.ser.write(line.encode('utf-8'))

    def read_steps(self):
        """
        Read a line from the serial port and attempt to decode it as a JSON object
        representing the current step positions.

        Behavior:
            - Reads and decodes a line from serial.
            - Attempts to parse JSON data to extract azimuth and elevation step positions.
            - Prints the positions, or the raw message if decoding fails.
        """
        raw = self.ser.readline().decode('utf-8').strip()
        try:
            data = json.loads(raw)
            print(f"Azimuth step={data['pos_az']}, Elevation step={data['pos_el']}")
        except json.JSONDecodeError:
            print('Received:', raw)

    def stop(self):
        """
        Send a stop command to the controller.

        Behavior:
            - Sends a JSON-encoded "STOP" message over serial to halt movement.
        """
        line = json.dumps("STOP") + "\n"
        self.ser.write(line.encode('utf-8'))
