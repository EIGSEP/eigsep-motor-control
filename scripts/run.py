from argparse import ArgumentParser
import logging
import time
from threading import Event, Thread
import eigsep_motor_control as emc

# Setup logging for information and debugging.
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

# Argument parsing setup to configure motor velocities and monitoring options.
parser = ArgumentParser(description="Control the motors")
parser.add_argument(
    "-a", "--az", type=int, default=0, help="Azimuth motor velocity"
)
parser.add_argument(
    "-e", "--el", type=int, default=0, help="Elevation motor velocity"
)
parser.add_argument(
    "-p", "--pot", action="store_true", help="Monitor potentiometer"
)
parser.add_argument(
    "-l", "--lim", action="store_true", help="Monitor limit switches"
)
args = parser.parse_args()

# Setting initial motor velocities from parsed arguments.
AZ_VEL = args.az
ALT_VEL = args.el

# Initialize events for reversing motor direction based on potentiometer monitoring.
az_reverse = Event()
alt_reverse = Event()
if args.pot:
    pot = emc.Potentiometer()
    pot_zero_count = 0 # Counter for consecutive zero-movement readings.
    pot_zero_reversed = False # Flag to track if reversal action has been taken.
    # Create and start a separate thread to monitor potentiometer if enabled.
    thd = Thread(
        target=pot.monitor, args=(az_reverse, alt_reverse), daemon=True
    )
    logging.info("Starting pot thread.")
    thd.start()
else:
    logging.warning("Potentiometers not being monitored.")

# Start the motors with the specified velocities.
logging.info(f"Starting motors with speeds: az={AZ_VEL}, alt={ALT_VEL}.")
motor = emc.Motor()
motor.start(az_vel=AZ_VEL, alt_vel=ALT_VEL)

# Initialize limit switch events if monitoring is enabled.
if args.lim:
    limits = [Event(), Event()]  # Events indicating limit switches are triggered.

try:
    while True:
        if args.pot:
            # Check if both motors show no movement and update the count.
            if pot.direction["az"] == 0 and pot.direction["alt"] == 0:
                pot_zero_count += 1
            else:
                pot_zero_count = 0
            # If 10 consecutive no-movement readings are detected, attempt to reverse motors.
            if pot_zero_count == 10: 
                if not pot_zero_reversed:
                    logging.info("No movement detected from az motor. Reversing az motor.")
                    motor.reverse("az")
                    time.sleep(0.25)
                    logging.info("No movement detected form alt motor. Reversing alt motor.")
                    motor.reverse("alt")
                    time.sleep(0.25)
                    pot_zero_reversed = True
                    pot_zero_count = 0
                else: 
                    # Stop the program after a second set of 10 no-movement readings post-reversal.
                    logging.info("No movement detected from motors. Exiting.")
                    break
            # Check and handle limit switch events.
            if args.lim:
                limits = emc.reverse_limit(motor, pot, limits)
        # Check and react to reverse signals set by potentiometer monitoring.
        if az_reverse.is_set():
            logging.info("Reversing az motor.")
            motor.reverse("az")
            time.sleep(0.25)
            az_reverse.clear()
        if alt_reverse.is_set():
            logging.info("Reversing alt motor.")
            motor.reverse("alt")
            time.sleep(0.25)
            alt_reverse.clear()
        time.sleep(0.1)
except KeyboardInterrupt:
    logging.info("Exiting.")
finally:
    # Ensure motors are stopped on exit.
    motor.stop()

# motor.stow(motors=["az", "alt"])