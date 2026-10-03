"""
Query or change the Tello's Wi-Fi AP SSID and/or password.

get_ssid()/set_ssid()/get_password()/set_password() on tellopy.Tello
block until acked (retrying a few times on timeout) and either return
the decoded value or raise tellopy.TelloError, so this script is just a
thin CLI wrapper.

IMPORTANT: a set only takes effect once the drone is power-cycled.
Reading the value back (or trying to join the AP) before that still
shows/uses the old one; this is not a bug in this script or in the
library, it is how the drone behaves (confirmed experimentally).

Usage:
    python3 -m tellopy.examples.wifi_config show
    python3 -m tellopy.examples.wifi_config ssid [<new_ssid>]
    python3 -m tellopy.examples.wifi_config password [<new_password>]
"""
import sys
import tellopy


def do_show(drone):
    print("ssid: %s" % drone.get_ssid())
    print("password: %s" % drone.get_password())


def do_ssid(drone, new_ssid):
    if new_ssid is None:
        print("ssid: %s" % drone.get_ssid())
    else:
        drone.set_ssid(new_ssid)
        print("ssid set to %r. Power-cycle the drone for it to take effect." % new_ssid)


def do_password(drone, new_password):
    if new_password is None:
        print("password: %s" % drone.get_password())
    else:
        drone.set_password(new_password)
        print("password set to %r. Power-cycle the drone for it to take effect." % new_password)


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ('show', 'ssid', 'password'):
        print(__doc__)
        sys.exit(1)

    drone = tellopy.Tello()
    drone.set_loglevel(drone.LOG_WARN)
    try:
        drone.connect()
        drone.wait_for_connection(60.0)

        if sys.argv[1] == 'show':
            do_show(drone)
        elif sys.argv[1] == 'ssid':
            do_ssid(drone, sys.argv[2] if len(sys.argv) >= 3 else None)
        elif sys.argv[1] == 'password':
            do_password(drone, sys.argv[2] if len(sys.argv) >= 3 else None)
    except Exception as ex:
        print(ex)
    finally:
        drone.quit()


if __name__ == '__main__':
    main()
