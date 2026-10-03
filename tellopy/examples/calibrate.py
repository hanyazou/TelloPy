"""
Run IMU calibration on a connected Tello.

This is a thin UI over tellopy.Tello's calibration support -- all the
protocol work happens in the library (see start_calibration() and
EVENT_CALIBRATION_STATUS in tellopy/_internal/tello.py). This script just
connects, starts calibration, and prints progress as it comes in.

You still have to physically move the drone through several distinct
orientations by hand while this runs -- see docs/calibration.md for the
procedure and safety notes (propellers off, flat surface, etc). The order
of orientations does not matter, only that each one is held still long
enough to be recognized (watch the drone's LED: yellow steady = still
calibrating, green fast blink = that orientation was just accepted).

Usage:
    python3 -m tellopy.examples.calibrate
"""
from time import sleep
import tellopy

_last_raw = [None]


def handler(event, sender, data, **args):
    drone = sender
    if event is drone.EVENT_CALIBRATION_STATUS:
        if data.raw != _last_raw[0]:
            _last_raw[0] = data.raw
            print(str(data))


def main():
    drone = tellopy.Tello()
    try:
        drone.set_loglevel(drone.LOG_WARN)
        drone.subscribe(drone.EVENT_CALIBRATION_STATUS, handler)
        drone.connect()
        drone.wait_for_connection(60.0)

        print("Starting IMU calibration.")
        print("See docs/calibration.md for the procedure if you haven't already.")
        drone.start_calibration()

        while drone.calibration_active:
            sleep(0.5)

        print("Calibration complete.")
    except Exception as ex:
        print(ex)
    finally:
        drone.quit()


if __name__ == "__main__":
    main()
