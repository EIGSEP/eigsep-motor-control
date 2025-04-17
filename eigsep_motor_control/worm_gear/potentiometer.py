import logging
import numpy as np
from pathlib import Path
import serial
import time
from threading import Event, Thread, Lock
import yaml
from eigsep_motor_control.serial_params import BAUDRATE, INT_LEN


class Potentiometer:

    NBITS = 16  # ADC number of bits
    VMAX = 3.3

    # serial connection constants (BAUDRATE defined in main.py)
    PORT = "/dev/ttyACM0"

    def __init__(self):
        """
        Class for reading voltages from the potentiometers.

        """
        self.ser = serial.Serial(port=self.PORT, baudrate=BAUDRATE)
        self.ser.reset_input_buffer()

        # voltage range of the pots
        path = Path(__file__).parent / "config.yaml"
        with open(path, "r") as f:
            config = yaml.safe_load(f)
        self.VOLT_RANGE = config["volt_range"]
        self.POT_ZERO_THRESHOLD = 0.0015

        # voltage measurements (az, alt)
        size = 5  # number of measurements to store XXX
        self.volts = np.zeros((size, 2))
        self.reset_volt_readings()

    @property
    def vdiff(self):
        """
        Find the difference between the last ``self.size'' voltage readings
        of each pot.

        Returns
        -------
        dict
            A dictionary containing the differences in voltage readings between
            the last two measurements for each pot. Keys are 'az' and 'alt'.

        """
        az, alt = np.diff(self.volts, axis=0).T
        return {"az": az, "alt": alt}

    @property
    def direction(self):
        """
        Determines direction of az/alt motors based on last ``self.size''
        voltage readings of the respective pot.

        """
        # XXX might need to adjust the size so that we can pick up change
        # of direction quickly enough
        d = {}
        for k, v in self.vdiff.items():
            x = np.mean(v)
            # the pot is considered stationary if changes are below threshold
            if np.abs(x) < self.POT_ZERO_THRESHOLD:
                d[k] = 0
            else:
                d[k] = int(np.sign(x))
        return d

    def bit2volt(self, analog_value):
        """
        Converts an analog value from bits to volts.

        Parameters
        -------
        analog_value : int
            The digital value to be converted to volts.

        Returns
        -------
        voltage : float
            The calculated voltage corresponding to the bit number.

        """
        res = 2**self.NBITS - 1
        voltage = (self.VMAX / res) * analog_value
        return voltage

    def read_analog(self):
        """
        Read the analog values of the pots.

        Returns
        -------
        data : np.ndarray
            The analog values of the pots averaged over INT_LEN
            measurements. The first value is associated with the azimuth
            pot, the second value is the altitude pot.

        """
        data = self.ser.readline().decode("utf-8").strip()
        data = [int(d) for d in data.split()]
        return np.array(data) / INT_LEN

    def read_volts(self, motor=None):
        """
        Read the current voltage from an analog sensor, converts it to volts,
        and updates the internal voltage history.

        Parameters
        -------
        motor : str, optional
            The motor identifier ('az' for azimuth or 'alt' for altitude) for
            which the voltage is to be returned. If no motor is specified, the
            voltage for both motors is returned.

        Returns
        -------
        v : float or np.ndarray
            The voltage reading for the specified motor, or an array of
            voltages if ``motor'' is None.

        """
        v = self.bit2volt(self.read_analog())
        self.volts = np.concatenate((self.volts[1:], v[None]), axis=0)
        if motor == "az":
            return v[0]
        elif motor == "alt":
            return v[1]
        else:
            return v

    def reset_volt_readings(self):
        """
        Read pot voltages quickly in succesion to reset the buffer. This
        is useful to get meaningful derivatives.

        """
        for i in range(self.volts.shape[0]):
            _ = self.read_volts()
            time.sleep(0.05)

    def _trigger_reverse(self, motor, volt_reading):
        """
        Continuously monitor the voltage levels of one pot to see if motor
        reaches its limit.

        Parameters
        ----------
        motor : str
            The motor to monitor. Either 'az' or 'alt'.
        volt_reading : float
            The current voltage reading of the motor.

        """
        vmin, vmax = self.VOLT_RANGE[motor]
        d = self.direction[motor]
        # check if the current voltage is outside the limits
        if d > 0 and volt_reading >= vmax:
            logging.warning(f"Pot {motor} at max voltage.")
            return True
        elif d < 0 and volt_reading <= vmin:
            logging.warning(f"Pot {motor} at min voltage.")
            return True
        else:
            return False

    def monitor(self, az_event, alt_event):
        """
        Continuously monitor the voltage levels of the 'az' (azimuth) and 'alt'
        (altitude) motors and checks these against predefined voltage ranges to
        trigger events if voltage limits are reached.

        Parameters
        ----------
        az_event : threading.Event
            An event triggered when the azimuth motor reaches its limit.
        alt_event : threading.Event
            An event triggered when the altitude motor reaches its limit.

        """
        names = []
        events = []
        if az_event is not None:
            names.append("az")
            events.append(az_event)
        if alt_event is not None:
            names.append("alt")
            events.append(alt_event)

        if not names:
            return

        while True:
            for m, event in zip(names, events):
                v = self.read_volts(motor=m)
                logging.info(f"{m}: {v:.3f} V ")
                trigger = self._trigger_reverse(m, v)
                if trigger:
                    event.set()
                    # self.reset_volt_readings()

class DummyPotentiometer(Potentiometer):
    def __init__(self, motor_system):
        # Manually initialize attributes needed from the base class
        self.lock = Lock()
        path = Path(__file__).parent / "config.yaml"
        with open(path, "r") as f:
            config = yaml.safe_load(f)
        self.VOLT_RANGE = config["dummy_volt_range"]
        self.POT_ZERO_THRESHOLD = 0.001
        # Voltage measurements (az, alt)
        size = 2  # Number of measurements to store
        self.volts = np.zeros((size, 2))
        self.motor_system = motor_system
        self.simulated_pots = {"az": 32768, "alt": 32768}  # Initial simulated mid-range pot values
        self.update_thread = Thread(target=self.update_pot_values, daemon=True)
        self.update_thread.start()
        self.reset_volt_readings()
        #print("DummyPotentiometer initialized with attributes:")
        #print(f"VOLT_RANGE: {self.VOLT_RANGE}")
        #print(f"POT_ZERO_THRESHOLD: {self.POT_ZERO_THRESHOLD}")
        #print(f"volts: {self.volts}")
        #print(f"simulated_pots: {self.simulated_pots}")


    def update_pot_values(self):
        """
        Continuously updates the simulated pot values based on motor velocities.
        """
        while True:
            time.sleep(0.5)  # Update frequency, adjust as needed
            with self.lock:
                for motor, speed in self.motor_system.velocities.items():
                    direction = np.sign(speed)
                    change = direction * np.abs(speed)
                    new_value = self.simulated_pots[motor] + change
                    # Clamp the values to stay within 16-bit range
                    self.simulated_pots[motor] = max(0, min(65535, new_value))

    def read_analog(self):
        """
        Simulate the reading of analog values from the pots based on current simulated values.
        """
        time.sleep(0.5)
        with self.lock:
            simulated_values = np.array([self.simulated_pots["az"], self.simulated_pots["alt"]])
        return simulated_values
