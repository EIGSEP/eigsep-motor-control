from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
import serial
import json
import time

parser = ArgumentParser(
    description="Motor velocity", formatter_class=ArgumentDefaultsHelpFormatter
)

parser.add_argument(
    "-t",
    dest="time",
    type=int,
    default=225,
    help="Delay between pulses sent to stepper motor in microseconds",
)
parser.add_argument(
    "-de",
    dest="degree_e",
    type=int,
    default=0,
    help="Number of degrees to rotate the box, (0, 360)",
)

parser.add_argument(
    "-da",
    dest="degree_a",
    type=int,
    default=0,
    help="Number of degrees to rotate the antenna, (0, 360)",
)

parser.add_argument(
    "-i",
    dest="inf_",
    type=int,
    default=0,
    help="Setting to rotate stepper motor infinitely",
)


parser.add_argument(
    "-s",
    dest="ser_",
    type=str,
    default="/dev/ttyACM0",
    help="USB serial port",
)

#parser.add_argument(
#    "-c",
#    dest="cancel_",
#    type=str,
#    default="c",
#    help="stops the motor",
#)

parser.add_argument('--c', action = 'store_true', help = 'Stops the motor')


args = parser.parse_args()
destination = args.ser_
delay = int(args.time)
box_angle = args.degree_e  # theta?
antenna_angle = args.degree_a  # phi?
inf = args.inf_

if args.c:
    ser = serial.Serial(destination, 115200, timeout=1)
    ser.write((json.dumps(["STOP"])).encode("utf-8"))


else:
    ser = serial.Serial(destination, 115200, timeout=1)
    #time.sleep(2)
    args = [delay, box_angle, antenna_angle, inf]
    print(args)
    ser.write((json.dumps(args) +"\n").encode("utf-8"))

#print('Ctrl C to stop motor...')
#try:
#    while True:
#        time.sleep(2)
#except KeyboardInterrupt:
#    ser.send_break()
#    ser.flush()
#    ser.close()

ser.flush()
ser.close()
