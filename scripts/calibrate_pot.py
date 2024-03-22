from argparse import ArgumentParser
import sys
import time
import numpy as np
import eigsep_motor_control as emc

parser = ArgumentParser(description="Calibrate potentiometers.")
parser.add_argument(
    "-a", "--az", action="store_true", help="Calibrate azimuth pot."
)
parser.add_argument(
    "-e", "--el", action="store_true", help="Calibrate elevation pot."
)
args = parser.parse_args()

motors = []
if args.az:
    motors.append("az")
    motors.append("az_reverse")
if args.el:
    motors.append("alt")
    motors.append("alt_reverse")
if not motors:
    print("Exiting...")
    sys.exit()

motor = emc.Motor()
pot = emc.Potentiometer()
vmax = {}

for m in motors:
    print(f"Calibrating {m.upper()} motor")
    if m == "az":
        az_vel = 254
        alt_vel = 0
    elif m == "alt":
        az_vel = 0
        alt_vel = 254
    elif m == "az_reverse":
        az_vel = -254
        alt_vel = 0
    elif m == "alt_reverse":
        az_vel = 0
        alt_vel = -254

    motor.start(az_vel=az_vel, alt_vel=alt_vel)
    try:
        volts = []
        while len(volts) < 2:
            volts.append(pot.read_volts(motor=m))
            time.sleep(0.1)
        # loops until the switch is triggered
        print("Attack mode.")
        while volts[-1] >= volts[-2]:  # XXX check the tolerance
            v = pot.read_volts(motor=m)
            print(v)
            volts.append(v)
            time.sleep(0.1)
        vmax[m] = np.max(volts)
        # now: either the pot is stuck or the switch is triggered
        while volts[-1] > vmax[m] - 0.1:  # XXX (somewhat) arbitrary threshold
            v = pot.read_volts(motor=m)
            print(v)
            volts.append(v)
            time.sleep(0.1)
        print("Triggered the switch.")
        # now we're certain that we're reversing
        # check if the pot was stuck
        if np.count_nonzero(np.abs(v - vmax[m]) < 0.01) > 2:  # XXX
            print("Got the pot stuck.")
            stuck = True
        else:
            print("Did not get the pot stuck.")
            stuck = False
        print("Reverse until switch is released!")
        while volts[-1] <= volts[-2]:
            v = pot.read_volts(motor=m)
            print(v)
            volts.append(v)
            time.sleep(0.1)
        if stuck:
            print("Calibration done.")
            motors.remove(f"{m}_reverse")  # don't need to do the reverse
        else:  # didn't get the pot stuck, run until the other end
            motor.stop()

    except KeyboardInterrupt:
        print("Interrupting.")
    finally:
        print("Stopping.")
        motor.stop()

print(vmax)
