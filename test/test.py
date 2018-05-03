from time import sleep
import tellopy
from tellopy._internal.utils import *

prev_flight_data = None


def handler(event, sender, data, **args):
    global prev_flight_data
    drone = sender
    if event is drone.CONNECTED_EVENT:
        print 'connected'
        drone.start_video()
        drone.set_exposure(0)
        drone.set_video_encoder_rate(4)
    elif event is drone.FLIGHT_EVENT:
        if prev_flight_data != str(data):
            print data
            prev_flight_data = str(data)
    elif event is drone.TIME_EVENT:
        print 'event="%s" data=%d' % (event.getname(), data[0] + data[1] << 8)
    elif event is drone.VIDEO_FRAME_EVENT:
        pass
    else:
        print 'event="%s" data=%s' % (event.getname(), str(data))


def test():
    drone = tellopy.Tello()
    try:
        # drone.set_loglevel(d.LOG_ALL)
        drone.subscribe(drone.CONNECTED_EVENT, handler)
        # drone.subscribe(drone.WIFI_EVENT, handler)
        # drone.subscribe(drone.LIGHT_EVENT, handler)
        drone.subscribe(drone.FLIGHT_EVENT, handler)
        # drone.subscribe(drone.LOG_EVENT, handler)
        drone.subscribe(drone.TIME_EVENT, handler)
        drone.subscribe(drone.VIDEO_FRAME_EVENT, handler)

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
    except Exception, ex:
        print ex
        show_exception(ex)
    finally:
        drone.quit()
    print 'end.'

if __name__ == '__main__':
    test()
