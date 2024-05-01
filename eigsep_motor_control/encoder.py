import logging
import numpy as np
import serial
import time
from qwiic_dual_encoder_reader import QwiicDualEncoderReader
from eigsep_motor_control.motor import MOTOR_ID
from eigsep_motor_control.serial_params import BAUDRATE, INT_LEN


class Encoder(QwiicDualEncoderReader):

    def get_encoder(self, motor):
        """
        Read the encoder count of a motor.

        Parameters
        ----------
        motor : str
            Either ``az'' or ``alt''. The motor to read the encoder value of.

        Returns
        -------
        encoder : int
            The encoder value corresponding to the given motor.

        """
        mid = MOTOR_ID[motor]
        if mid == 0:
            return self.encoder.count1
        elif mid == 1:
            return self.encoder.count2


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
        self.VOLT_RANGE = {"az": (0.3, 2.5), "alt": (1., 2.)}
        self.POT_ZERO_THRESHOLD = 0.005

        # voltage measurements (az, alt)
        size = 3  # number of measurements to store XXX
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
                d[k] = np.sign(x)
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
                print(f"{m}: {v:.3f} V ")
                trigger = self._trigger_reverse(m, v)
                if trigger:
                    event.set()
                    self.reset_volt_readings()
