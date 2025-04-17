from argparse import ArgumentParser
import logging
import numpy as np
from pathlib import Path
import time
import yaml
import eigsep_motor_control as emc


def calibrate(motor, m, direction):
    """
    Calibrate the potentiometer corresponding to the motor.

    Parameters
    ----------
    motor : str
        Name of motor to calibrate. Either 'az' or 'alt'.
    m : emc.Motor
        Instance of the emc.Motor class.
    direction : int
        Direction of the motor. 1 for forward (increasing pot voltages),
        -1 for reverse (decreasing pot voltages).

    Returns
    -------
    vm : float
        Maximum/minimum pot voltage (depending on the direction).
    stuck : bool
        True if the pot was stuck, False otherwise.

    """
    pot = emc.Potentiometer()

    if direction == -1:
        vel = m.MIN_SPEED
    elif direction == 1:
        vel = m.MAX_SPEED
    else:
        raise ValueError("Invalid direction, must be -1 or 1.")
    if motor == "az":
        az_vel = vel
        alt_vel = 0
    elif motor == "alt":
        az_vel = 0
        alt_vel = vel
    else:
        raise ValueError("Invalid motor, must be ``az'' or ``alt''.")
    m.set_velocity(az_vel=az_vel, alt_vel=alt_vel)
    m.logger.warning("Attack mode.")
    pot.reset_volt_readings()
    # check if limit switch is already pulled at start-up and undo it
    while pot.direction[motor] == -direction:
        m.logger.warning("Limit switch is triggered, reversing")
        v = pot.read_volts(motor=motor)
        time.sleep(0.1)

    # loops until the switch is triggered
    last_motion = time.time()
    while pot.direction[motor] == direction:
        print(pot.direction[motor], direction)
        v = pot.read_volts(motor=motor)
        m.logger.info(f"{v=:.3f}")
        time.sleep(0.1)
    # get the extremum pot voltage (max if forward, min if reverse)
    vm = np.max(np.abs(pot.volts[:, emc.motor.MOTOR_ID[motor]]))
    m.logger.info(f"Extremum voltage: {vm:.3f}")
    while pot.direction[motor] == 0:  # pot stuck
        v = pot.read_volts(motor=motor)
        m.logger.info(f"{v=:.3f}")
        time.sleep(0.01)
    m.logger.info("Reverse until switch is released.")
    while pot.direction[motor] == -direction:
        v = pot.read_volts(motor=motor)
        m.logger.info(f"{v=:.3f}")
        time.sleep(0.01)

    m.stop()  # stop in attacking mode
    return vm


if __name__ == "__main__":

    logger = logging.getLogger(__name__)
    logging.basicConfig(level=logging.INFO)
    parser = ArgumentParser(description="Calibrate potentiometers.")
    parser.add_argument(
        "-b",
        "--board",
        type=str,
        default="pololu",
        help="Motor board to use: ``pololu'' (default) or ``qwiic''.",
    )
    parser.add_argument(
        "-a", "--az", action="store_true", help="Calibrate azimuth pot."
    )
    parser.add_argument(
        "-e", "--el", action="store_true", help="Calibrate elevation pot."
    )
    args = parser.parse_args()

    DELTA = 0.1  # diff between pot limits and hard limit

    motors = []
    if args.az:
        motors.append("az")
    if args.el:
        motors.append("alt")
    if not motors:
        raise ValueError("At least one motor must be selected.")

    if args.board == "pololu":
        m = emc.PololuMotor(logger=logger)
    elif args.board == "qwiic":
        m = emc.QwiicMotor(logger=logger)
    else:
        raise ValueError("Invalid board, must be ``pololu'' or ``qwiic''.")

    path = Path(__file__).parent.parent / "eigsep_motor_control" / "config.yaml"
    with open(path, "r") as f:
        config = yaml.safe_load(f)
        volt_range = config["volt_range"]
    for motor in motors:
        # voltage difference between min and max pot voltage
        vdiff = volt_range[motor][1] - volt_range[motor][0]
        logger.info(f"Calibrating {motor} potentiometer.")
        vmax = calibrate(motor, m, 1)
        vmin = calibrate(motor, m, -1)
        volt_range[motor] = [float(vmin)+DELTA, float(vmax)-DELTA]
    config["volt_range"] = volt_range
    with open(path, "w") as f:
        yaml.safe_dump(config, f)
    logger.warning(
        "Calibration successful, config file updated, run ``sudo pip3 install"
        " .'' to apply changes."
    )
