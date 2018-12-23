from time import sleep
import tellopy
import datetime
import os

file = None
write_header = True

def handler(event, sender, data, **args):
    global file
    global write_header
    drone = sender
    if event is drone.EVENT_LOG_DATA:
        if file == None:
            path = '%s/Desktop/tello-%s.csv' % (
                os.getenv('HOME'),
                datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S'))
            file = open(path, 'w')
        if write_header:
            file.write('%s\n' % data.format_cvs_header())
            write_header = False
        file.write('%s\n' % data.format_cvs())
    if event is drone.EVENT_FLIGHT_DATA or event is drone.EVENT_LOG_DATA:
        print('record_log: %s: %s' % (event.name, str(data)))

def test():
    drone = tellopy.Tello()
    try:
        drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)
        drone.subscribe(drone.EVENT_LOG_DATA, handler)
        drone.record_log_data()

        drone.connect()
        drone.wait_for_connection(60.0)
        drone.takeoff()
        sleep(5)
        drone.clockwise(100)
        sleep(5)
        drone.clockwise(0)
        drone.down(50)
        sleep(2)
        drone.up(50)
        sleep(2)
        drone.up(0)
        drone.land()
        sleep(5)
    except Exception as ex:
        print(ex)
    finally:
        drone.quit()

if __name__ == '__main__':
    test()
