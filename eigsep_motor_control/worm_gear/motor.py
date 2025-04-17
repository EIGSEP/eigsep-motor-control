import logging
import numpy as np
import time
from threading import Event, Thread, Lock
try:
    from RPi import GPIO
except ImportError:
    pass
from qwiic_scmd import QwiicScmd

MOTOR_ID = {"az": 0, "alt": 1}
# min/max speeds for each motor driver
MIN_SPEED = {"pololu": -480, "qwiic": -255, "dummy": -250}
MAX_SPEED = {"pololu": 480, "qwiic": 254, "dummy": 250}

#XXX alt motor is wired with opposite polarity so being reversed in 
# motor.set+velocoty

class Motor:
    def __init__(self, logger=None):
        self.velocities = {"az": 0, "alt": 0}
        self.debounce_interval = 5  # debounce interval in seconds
        # last reversal timestamps for motors
        self.last_reversal_time = {
            "az": -self.debounce_interval, "alt": -self.debounce_interval
        }
        self.limit_reversal = False

        # set up logging
        if logger is None:
            logger = logging.getLogger(__name__)
            logger.basicConfig(level=logging.INFO)
        self.logger = logger

    def set_velocity(self, az_vel, alt_vel):
        """Starts both motors with the given velocities."""
        raise NotImplementedError("Method must be implemented by subclass.")

    def should_reverse(self, motor):
        """Determine if the motor should reverse."""
        return (
            time.time() - self.last_reversal_time[motor]
            > self.debounce_interval
        )

    def reverse(self, motor, force=False):
        """
        Reverse the direction of the specified motor unless the motor
        has been reversed too recently (can be forced with the force flag).

        Parameters
        ----------
        motor : str
            The motor to reverse. Must be either "az" or "alt".
        force : bool
            Reverse the motor regardless of the debounce interval.

        """
        if not self.should_reverse(motor) and not force:
            self.logger.warning("Reversal too soon after last reversal.")
            return
        if motor == "az":
            az_vel = -1 * self.velocities["az"]
            alt_vel = self.velocities["alt"]
        elif motor == "alt":
            az_vel = self.velocities["az"]
            alt_vel = -1 * self.velocities["alt"]
        else:
            raise ValueError("Invalid motor specified.")
        self.set_velocity(az_vel, alt_vel)
        if not force:
            self.last_reversal_time[motor] = time.time()

    def stop(self, motors=("az", "alt")):
        """
        Stop the specified motors (azimuth and/or altitude).

        Parameters
        ----------
        motors : str or list of str
            The motor(s) to stop. Must be either "az" or "alt" or a list of
            these strings.

        """
        if isinstance(motors, str):
            motors = [motors]
        vel = self.velocities
        for m in motors:
            vel[m] = 0
        self.set_velocity(vel["az"], vel["alt"])

    def stow(self, motors=("az", "alt")):
        """Return motors to a home position."""
        raise NotImplementedError("Stow method not implemented yet.")

    def cleanup(self):
        self.stop()
        # self.stow()


class QwiicMotor(Motor, QwiicScmd):

    def __init__(self, logger=None, address=None, i2c_driver=None):
        Motor.__init__(self, logger=logger)
        QwiicScmd.__init__(self, address=address, i2c_driver=i2c_driver)
        self.MIN_SPEED = MIN_SPEED["qwiic"]
        self.MAX_SPEED = MAX_SPEED["qwiic"]
        assert self.begin(), "Initalization of SCMD failed."
        self.enable()

    def set_velocity(self, az_vel, alt_vel):
        """Sets the velocity of each motor."""
        self.velocities = {"az": az_vel, "alt": -alt_vel}  #XXX
        for m, v in self.velocities.items():
            if v < self.MIN_SPEED:
                v = self.MIN_SPEED
                self.logger.warning(
                    f"Speed for {m} motor too low. Setting to {v}."
                )
            elif v > self.MAX_SPEED:
                v = self.MAX_SPEED
                self.logger.warning(
                    f"Speed for {m} motor too high. Setting to {v}."
                )
            speed = np.abs(v)
            direction = 1 if v > 0 else 0
            self.set_drive(MOTOR_ID[m], direction, speed)


class PololuMotor(Motor):

    # gpio pins for each motor (az/alt), for speed (PWM) and direction
    PWM_PINS = {"az": 12, "alt": 13}
    DIR_PINS = {"az": 24, "alt": 25}
    EN_PIN = 5  # set to LOW to enable motors, HIGH to disable
    FAULT_PIN = 6  # normally HIGH, goes LOW when there is a fault

    def __init__(self, pwm_frequency=20e3, logger=None):
        super().__init__(logger=logger)
        self.MIN_SPEED = MIN_SPEED["pololu"]
        self.MAX_SPEED = MAX_SPEED["pololu"]
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        # setup all pins as output
        GPIO.setup(list(self.PWM_PINS.values()), GPIO.OUT)
        GPIO.setup(list(self.DIR_PINS.values()), GPIO.OUT)
        GPIO.setup(self.EN_PIN, GPIO.OUT)
        # we first set the fault pin as output to ensure it is HIGH
        GPIO.setup(self.FAULT_PIN, GPIO.OUT)
        GPIO.output(self.FAULT_PIN, GPIO.HIGH)
        # now we set it as input
        GPIO.setup(self.FAULT_PIN, GPIO.IN)
        self.enable()
        # set up PWM for speed control
        self.pwm = {}
        if pwm_frequency > 50e3:
            self.logger.warning("PWM frequency too high, setting to 50 kHz.")
            pwm_frequency = 50e3
        self.pwm_frequency = pwm_frequency
        for m, pin in self.PWM_PINS.items():
            self.pwm[m] = GPIO.PWM(pin, self.pwm_frequency)
            self.pwm[m].start(0)

    def enable(self):
        """Enable the motor driver."""
        GPIO.output(self.EN_PIN, GPIO.LOW)

    def disable(self):
        """Disable the motor driver."""
        GPIO.output(self.EN_PIN, GPIO.HIGH)

    def fault(self):
        """Check if there is a fault with the motor driver."""
        return GPIO.input(self.FAULT_PIN) == GPIO.LOW

    def change_pwm_frequency(self, frequency):
        """Change the PWM frequency of the motor driver."""
        if frequency > 50e3:
            raise ValueError("PWM frequency too high, max is 50 kHz.")
        for m, pwm in self.pwm.items():
            pwm.ChangeFrequency(frequency)
        self.pwm_frequency = frequency

    def set_velocity(self, az_vel, alt_vel):
        """Sets the velocity of each motor."""
        self.velocities = {"az": az_vel, "alt": -alt_vel}  #XXX
        for m, v in self.velocities.items():
            if v < self.MIN_SPEED:
                v = self.MIN_SPEED
                self.logger.warning(
                    f"Speed for {m} motor too low. Setting to {v}."
                )
            elif v > self.MAX_SPEED:
                v = self.MAX_SPEED
                self.logger.warning(
                    f"Speed for {m} motor too high. Setting to {v}."
                )
            speed = np.abs(v)
            # NOTE: annoyingly, this direction convention is opposite of the
            # other motor board
            direction = 0 if v > 0 else 1
            self.set_drive(m, direction, speed)

    def _speed2dc(self, speed):
        """Convert speed to duty cycle for PWM."""
        return np.abs(speed) / self.MAX_SPEED * 100

    def set_drive(self, motor, direction, speed):
        """
        Drive a motor at a given speed. Users should call the set_velocity
        method, not this one.

        Parameters
        ----------
        motor : str
            The motor to drive, must be ``az'' (azimuth) or ``alt'' (altitude)
        direction : int
            Direction to drive motor in, must be 0 (``forward'', i.e.,
            increasing pot voltage) or 1 (``backward'').
        speed : int
            The unsigned speed to drive the motor at. Must be between 0 and
            ``MAX_SPEED''.

        """
        GPIO.output(self.DIR_PINS[motor], direction)
        duty_cycle = self._speed2dc(speed)
        self.pwm[motor].ChangeDutyCycle(duty_cycle)

    def cleanup(self):
        self.stop()
        # self.stow()
        for pwm in self.pwm.values():
            pwm.stop()
        self.disable()
        GPIO.cleanup()

class DummyMotor(Motor):
    def __init__(self, logger=None):
        super().__init__(logger)
        self.MIN_SPEED = MIN_SPEED["dummy"]
        self.MAX_SPEED = MAX_SPEED["dummy"]
        self.simulated_positions = {"az": 0, "alt": 0}  # Initial positions for azimuth and altitude
        self.position_limits = {"az": (-15000, 15000), "alt": (-15000, 15000)}  # Position limits for each motor
        self.limit_reversal_time = False
        self.update_thread = None
        self.running = False

    def start_updates(self):
        """
        Start the background thread for updating motor positions.
        """
        if not self.running:
            self.running = True
            self.update_thread = Thread(target=self.update_positions, daemon=True)
            self.update_thread.start()

    def set_velocity(self, az_vel, alt_vel):
        """
        Thread-safely set the velocity for both motors and log the action.
        """
        if not self.running:
            self.start_updates()
        self.velocities = {"az": az_vel, "alt": alt_vel}
        for m, v in self.velocities.items():
            if v < self.MIN_SPEED:
                v = self.MIN_SPEED
                self.logger.warning(
                    f"Speed for {m} motor too low. Setting to {v}."
                )
            elif v > self.MAX_SPEED:
                v = self.MAX_SPEED
                self.logger.warning(
                    f"Speed for {m} motor too high. Setting to {v}."
                )
            if m == "az":
                self.velocities["az"] = v
                az_vel = v
            elif m == "alt":
                self.velocities["alt"] = v
                alt_vel = v
        self.logger.info(f"DummyMotor: Set velocities to azimuth: {az_vel} and altitude: {alt_vel}")

    def update_positions(self):
        """
        Continuously update the positions of the motors based on their velocities.
        This method runs in a background thread.
        """
        check_time = time.time() - 2
        while self.running:
            time.sleep(self.update_interval())
            for motor in ['az', 'alt']:
                old_position = self.simulated_positions[motor]
                displacement = self.velocities[motor] * self.update_interval()
                new_position = old_position + displacement
                min_limit, max_limit = self.position_limits[motor]
                # Check for limit switch activation
                if (new_position <= min_limit or new_position >= max_limit) and not self.limit_reversal and not self.limit_reversal_time:
                    if time.time() > check_time + 1:
                        self.limit_reversal_time = True
                        check_time = time.time()
                elif (new_position <= min_limit or new_position >= max_limit) and not self.limit_reversal:
                    # Reverse the velocity
                    if time.time() > check_time + 1:
                        self.reverse(motor, True)
                        self.limit_reversal = True
                        #print(self.limit_reversal)
                        self.logger.info("DummyMotor: Hit limit switch, motors manually reversing.")
                        check_time = time.time()
                elif (new_position >= min_limit and new_position <= max_limit) and self.limit_reversal:
                    if time.time() > check_time + 1:
                        self.logger.info("DummyMotor: Untriggered limit switch, motors manually reversing.")
                        self.reverse(motor, True)
                        self.limit_reversal = False
                        self.limit_reversal_time = False
                        check_time = time.time()

                self.simulated_positions[motor] = new_position

    def stop_updates(self):
        """
        Stop the background thread updating motor positions.
        """
        if self.running:
            self.running = False
            if self.update_thread is not None:
                self.update_thread.join()
                self.logger.info("DummyMotor: Stopped the update thread.")

    def update_interval(self):
        """
        Provides a time interval for updates. In a real application, this might be tied to a timer or the system clock.
        """
        return 0.01  # 10 ms update interval for high-frequency updates

    def stop(self, motors=("az", "alt")):
        super().stop(motors)
        for motor in motors:
            self.logger.info(f"DummyMotor: Stopped {motor} motor at position {self.simulated_positions[motor]}.")

    def cleanup(self):
        self.stop()
        self.stop_updates()
        super().cleanup()
        self.logger.info("DummyMotor: Cleaned up resources.")
