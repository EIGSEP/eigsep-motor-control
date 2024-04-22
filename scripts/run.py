from argparse import ArgumentParser
import logging
import time
from threading import Event, Thread
import eigsep_motor_control as emc

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

parser = ArgumentParser(description="Control the motors")
parser.add_argument("-a", "--az", type=int, help="Azimuth motor velocity")
parser.add_argument("-e", "--el", type=int, help="Elevation motor velocity")
parser.add_argument(
    "-p", "--pot", action="store_true", help="Monitor potentiometer"
)
args = parser.parse_args()

# motor velocities
AZ_VEL = args.az if args.az else 0
ALT_VEL = args.el if args.el else 0

# monitor potentiometer
# create events that tells motors to reverse direction
az_reverse = Event()
alt_reverse = Event()
if args.pot:
    pot = emc.Potentiometer()
    # thread monitoring potentiometer voltage and setting events
    thd = Thread(
        target=pot.monitor, args=(az_reverse, alt_reverse), daemon=True
    )
    logging.info("Starting pot thread.")
    thd.start()
else:
    logging.warning("Potentiometers not being monitored.")

# start motors
logging.info(f"Starting motors with speeds: az={AZ_VEL}, alt={ALT_VEL}.")
motor = emc.Motor()
motor.start(az_vel=AZ_VEL, alt_vel=ALT_VEL)


limits = [Event(), Event()]  # events indicating limit switches are hit

try:
    while True:
        limits = emc.reverse_limit(motor, pot, limits)
        if az_reverse.is_set():
            logging.info("Reversing az motor.")
            motor.reverse("az")
            time.sleep(0.25)
            # pot.reset_volt_readings()
            az_reverse.clear()
        if alt_reverse.is_set():
            logging.info("Reversing alt motor.")
            motor.reverse("alt")
            time.sleep(0.25)
            # pot.reset_volt_readings()
            alt_reverse.clear()
        time.sleep(0.1)
except KeyboardInterrupt:
    logging.info("Exiting.")
finally:
    motor.stop()

# motor.stow(motors=["az", "alt"])
