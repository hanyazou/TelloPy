from time import sleep
import tellopy


def handler(event, sender, data, **args):
    drone = sender
    if event is drone.CONNECTED_EVENT:
        print('connected')
    elif event is drone.FLIGHT_EVENT:
        print(data)


def test():
    drone = tellopy.Tello()
    try:
        drone.subscribe(drone.CONNECTED_EVENT, handler)
        drone.subscribe(drone.FLIGHT_EVENT, handler)

        drone.connect()
        sleep(2)
        drone.takeoff()
        sleep(5)
        drone.down(50)
        sleep(5)
        drone.land()
        sleep(5)
    except Exception as ex:
        print(ex)
    finally:
        drone.quit()

if __name__ == '__main__':
    test()
