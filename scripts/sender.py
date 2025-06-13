#!/usr/bin/env python3
import argparse
import serial
import sys
import os
import time
import json
import signal
import threading

# conversion constants
STEP_ANGLE = 1.8      # degrees per full step
MICROSTEP  = 4        # microsteps per full step
GEAR_TEETH = 113      # teeth count

DEFAULT_DELAY    = 225
DEFAULT_DEG_E    = 0
DEFAULT_DEG_A    = 0
DEFAULT_REPORT   = 100
DEFAULT_SERIAL   = "/dev/ttyACM0"
DEFAULT_MAX_SIZE = 0  # no rotation


class Sender:
    def __init__(self, device, delay, deg_e, deg_a, report, max_size, observe):
        self.device   = device
        self.delay    = delay
        self.deg_e    = deg_e
        self.deg_a    = deg_a
        self.report   = report
        self.max_size = max_size
        self.observe  = observe

        # instance‐level state
        self.stop_flag = False
        self.ser       = None   # serial.Serial instance
        self.idx       = 0
        self.off_az    = 0
        self.off_el    = 0

        # set up Ctrl+C handler bound to this instance
        signal.signal(signal.SIGINT, self._on_sigint)

    def _on_sigint(self, signum, frame):
        """
        Called when user hits Ctrl+C.  Raises stop_flag and immediately
        sends ["STOP"] to the Pico.
        """
        if not self.stop_flag and self.ser:
            self.stop_flag = True
            msg = json.dumps(["STOP"]) + "\n"
            try:
                self.ser.write(msg.encode("utf-8"))
                self.ser.flush()
                print("\n[sender] Emergency STOP sent via Ctrl+C")
            except Exception:
                pass  # ignore any write errors here

    @staticmethod
    def calc_pulses(deg):
        return int(MICROSTEP * GEAR_TEETH * abs(deg) / STEP_ANGLE)

    def scan_combined(self):
        """
        Scans the current directory for "combined_step_log.txt" or
        "combined_step_log_N.txt" to find:
          - highest index N
          - last recorded az/el offsets
        Returns (idx, off_az, off_el).  If none exist, returns (0, 0, 0).
        """
        max_idx = -1
        last_file = None
        for entry in os.listdir("."):
            if entry == "combined_step_log.txt":
                if max_idx < 0:
                    max_idx = 0
                    last_file = entry
            else:
                # look for "combined_step_log_<N>.txt"
                parts = entry.rsplit("_", 2)
                if len(parts) == 3 and parts[0] == "combined_step_log" and parts[2] == "txt":
                    try:
                        n = int(parts[1])
                        if n > max_idx:
                            max_idx = n
                            last_file = entry
                    except ValueError:
                        pass

        if max_idx < 0:
            return 0, 0, 0

        off_az = 0
        off_el = 0
        try:
            with open(last_file, "r") as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    parts = line.split(",")
                    if len(parts) != 3:
                        continue
                    try:
                        a = int(parts[1])
                        e = int(parts[2])
                    except ValueError:
                        continue
                    off_az = a
                    off_el = e
        except FileNotFoundError:
            off_az = off_el = 0

        return max_idx, off_az, off_el

    @staticmethod
    def rotate_log_if_needed(idx, max_size):
        """
        If max_size > 0, checks the size of "combined_step_log_<idx>.txt".
        If ≥ max_size, returns idx+1, else returns idx unchanged.
        If max_size == 0, always returns 0 (the base file).
        """
        if max_size <= 0:
            return 0
        filename = f"combined_step_log_{idx}.txt"
        if os.path.exists(filename):
            if os.path.getsize(filename) >= max_size:
                return idx + 1
        return idx

    @staticmethod
    def get_log_filename(idx):
        return "combined_step_log.txt" if idx == 0 else f"combined_step_log_{idx}.txt"

    def do_move(self, motor_name, deg):
        """
        Sends one move command (motor_name, deg).  Reads back exactly
        floor(pulses/report) + 1 STATUS lines and logs them with
        timestamp,az,el into the combined log file.

        Updates self.idx, self.off_az, self.off_el accordingly.
        Returns True if we hit stop_flag, False otherwise.
        """
        pulses = self.calc_pulses(deg)
        direction = 1 if deg >= 0 else -1
        is_az = (motor_name == "azimuth")

        # Update (and possibly rotate) the log file index
        self.idx = self.rotate_log_if_needed(self.idx, self.max_size)
        log_fn = self.get_log_filename(self.idx)

        # Number of STATUS lines to expect
        expected = pulses // self.report + 1
        seen = 0

        try:
            lf = open(log_fn, "a+")
        except OSError as ex:
            print(f"[sender] Error opening log file {log_fn}: {ex}", file=sys.stderr)
            return True  # bail

        # Send the JSON command over serial
        cmd = {
            "delay": self.delay,
            "pulses": pulses,
            "dir": direction,
            "report": self.report,
            "motor": 1 if is_az else 0
        }
        msg = json.dumps(cmd) + "\n"
        try:
            self.ser.write(msg.encode("utf-8"))
            self.ser.flush()
        except Exception as ex:
            print(f"[sender] Serial write error: {ex}", file=sys.stderr)
            lf.close()
            return True

        # Read STATUS lines until we've seen enough or stop_flag
        while not self.stop_flag and seen < expected:
            line = self.ser.readline().decode("utf-8", errors="ignore")
            if not line:
                break
            line = line.strip()
            if "EMERGENCY STOP" in line:
                self.stop_flag = True
                break
            if line.startswith("STATUS"):
                # Format: "STATUS <az>,<el>"
                parts = line.split()
                if len(parts) != 2:
                    continue
                try:
                    a_str, e_str = parts[1].split(",")
                    a = int(a_str)
                    e = int(e_str)
                except ValueError:
                    continue

                ts = int(time.time() * 1_000_000)  # microsecond timestamp
                lf.write(f"{ts},{a},{e}\n")
                lf.flush()
                seen += 1

        # Update offsets
        if is_az:
            self.off_az += pulses * direction
        else:
            self.off_el += pulses * direction

        lf.close()
        return self.stop_flag

    def run(self):
        """
        Main entry point after parsing arguments.  Opens serial,
        starts either individual moves or observe loop, then closes.
        """
        # Immediate STOP mode (if user passed -c/--stop)
        if self.args_stop:
            try:
                with serial.Serial(self.device, 115200, timeout=1) as s:
                    s.write((json.dumps(["STOP"]) + "\n").encode("utf-8"))
                    s.flush()
            except serial.SerialException as ex:
                print(f"[sender] Error opening {self.device}: {ex}", file=sys.stderr)
            return

        # Open the serial port
        try:
            self.ser = serial.Serial(self.device, 115200, timeout=1)
        except serial.SerialException as ex:
            print(f"[sender] Could not open serial port {self.device}: {ex}", file=sys.stderr)
            return

        # Initialize idx/off_az/off_el based on existing logs
        self.idx, self.off_az, self.off_el = self.scan_combined()

        # If observe‐mode, run the az+/el+/az−/el+ cycle until stopped
        if self.observe:
            moved = 0
            dir_e = 1
            while not self.stop_flag:
                # Az +360°
                self.do_move("azimuth", 360)
                if self.stop_flag: break

                # Elevation +10°
                moved += 10 * dir_e
                self.do_move("elevation", 10 * dir_e)
                if self.stop_flag: break

                # Az −360°
                self.do_move("azimuth", -360)
                if self.stop_flag: break

                # Elevation +10° (same dir_e)
                moved += 10 * dir_e
                self.do_move("elevation", 10 * dir_e)
                if self.stop_flag: break

                # After 360° total, reverse elevation direction
                if abs(moved) >= 360:
                    dir_e = -dir_e
                    moved = 0

        else:
            # Individual moves
            if self.deg_a != 0:
                self.do_move("azimuth", self.deg_a)
            if self.deg_e != 0:
                self.do_move("elevation", self.deg_e)

        self.ser.close()


def main():
    parser = argparse.ArgumentParser(description="Motor sender (Python, no globals)")
    parser.add_argument("-t", "--time", dest="delay", type=int, default=DEFAULT_DELAY,
                        help="Delay between pulses in microseconds")
    parser.add_argument("-e", "--degree_e", dest="deg_e", type=int, default=DEFAULT_DEG_E,
                        help="Elevation change (°)")
    parser.add_argument("-a", "--degree_a", dest="deg_a", type=int, default=DEFAULT_DEG_A,
                        help="Azimuth change (°)")
    parser.add_argument("-r", "--report", dest="report", type=int, default=DEFAULT_REPORT,
                        help="Report every N steps")
    parser.add_argument("-m", "--max-size", dest="max_size", type=int, default=DEFAULT_MAX_SIZE,
                        help="Rotate combined log when size ≥ this many bytes")
    parser.add_argument("-s", "--serial", dest="device", type=str, default=DEFAULT_SERIAL,
                        help="Serial port (e.g. /dev/ttyACM0)")
    parser.add_argument("-c", "--stop", action="store_true", dest="args_stop",
                        help="Send STOP immediately and exit")
    parser.add_argument("-o", "--observe", action="store_true", dest="observe",
                        help="Run full observe cycle")
    args = parser.parse_args()

    sender = Sender(
        device   = args.device,
        delay    = args.delay,
        deg_e    = args.deg_e,
        deg_a    = args.deg_a,
        report   = args.report,
        max_size = args.max_size,
        observe  = args.observe
    )
    sender.args_stop = args.args_stop
    sender.run()


if __name__ == "__main__":
    main()

