import numpy as np
from qwiic_scmd import QwiicScmd

MOTOR_ID = {"az": 0, "alt": 1}


class Motor(QwiicScmd):

    def __init__(self):
        """
        Class for controlling and monitoring DC motors using Sparkfun AutoPHat
        and associated software.

        """
        super().__init__(address=None, i2c_driver=None)
        assert self.begin(), "Initalization of SCMD failed."
        for i in [0, 1]:
            self.set_drive(i, 0, 0)
        self.enable()
        # current velocities of the form (direction, speed):
        self.velocities = {"az": (0, 0), "alt": (0, 0)}

    def start(self, az_vel=254, alt_vel=254):
        """
        Start one or both motors with the given velocities. The given motor
        will not start if the speed is set to 0.

        Parameters
        ----------
        az_vel : int
            The velocity of the azimuthal motor. Positive values indicate
            clockwise rotation as seen from the top. Must be in the
            range [-255, 254].
        alt_vel : int
            Same as ``speed_az'' for the altitude motor.

        """
        velocities = {"az": az_vel, "alt": alt_vel}
        for m, v in velocities.items():
            speed = np.abs(v)
            if speed == 0:
                continue
            direction = 1 if v > 0 else 0
            self.set_drive(MOTOR_ID[m], direction, speed)
            self.velocities[m] = (direction, speed)

    def reverse(self, motor):
        """
        Reverse one of the motors.

        Parameters
        ----------
        motor : str
            Indicate which motor to reverse. Must be ``az'' or ``alt''.

        """
        d, s = self.velocities[motor]
        reverse_dir = (d + 1) % 2  # turn 0 to 1 and vice versa
        self.set_drive(MOTOR_ID[motor], reverse_dir, s)
        self.velocities[motor] = (reverse_dir, s)

    def stop(self, motors=["az", "alt"]):
        """
        Stop one or both motors.

        Parameters
        ----------
        motors : str or list of str
            The motor(s) to stop. Valid names are ``az'' and ``alt'' for the
            azimuth and altitude motors, respectively.

        """
        if isinstance(motors, str):
            motors = [motors]
        for m in motors:
            self.set_drive(MOTOR_ID[m], 0, 0)

    def stow(self, motors=["az", "alt"]):
        """Return to home."""
        raise NotImplementedError
