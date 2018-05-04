from time import sleep
import tellopy
from tellopy._internal.utils import *

prev_flight_data = None


def handler(event, sender, data, **args):
    global prev_flight_data
    drone = sender
    if event is drone.EVENT_CONNECTED:
        print('connected')
        drone.start_video()
        drone.set_exposure(0)
        drone.set_video_encoder_rate(4)
    elif event is drone.EVENT_FLIGHT_DATA:
        if prev_flight_data != str(data):
            print(data)
            prev_flight_data = str(data)
    elif event is drone.EVENT_TIME:
        print('event="%s" data=%d' % (event.getname(), data[0] + data[1] << 8))
    elif event is drone.EVENT_VIDEO_FRAME:
        pass
    else:
        print('event="%s" data=%s' % (event.getname(), str(data)))


def test():
    drone = tellopy.Tello()
    try:
        # drone.set_loglevel(d.LOG_ALL)
        drone.subscribe(drone.EVENT_CONNECTED, handler)
        # drone.subscribe(drone.EVENT_WIFI, handler)
        # drone.subscribe(drone.EVENT_LIGHT, handler)
        drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)
        # drone.subscribe(drone.EVENT_LOG, handler)
        drone.subscribe(drone.EVENT_TIME, handler)
        drone.subscribe(drone.EVENT_VIDEO_FRAME, handler)

        drone.connect()
        # drone.takeoff()
        # time.sleep(5)
        drone.down(50)
        sleep(3)
        drone.up(50)
        sleep(3)
        drone.down(0)
        sleep(2)
        drone.land()
        sleep(5)
    except Exception as ex:
        print(ex)
        show_exception(ex)
    finally:
        drone.quit()
    print('end.')

if __name__ == '__main__':
    test()
