"""
tellopy sample using keyboard and video player

 - you must install mplayer to replay the video
 - WASD + shift/space to translate the drone
 - QE or arrows to yaw
 - tab to lift off, backspace to land
"""

import time
import sys
import tellopy
import pygame
import pygame.display
import pygame.key
import pygame.locals
from subprocess import Popen, PIPE

prev_flight_data = None
video_player = None

controls = {
    'w': 'forward',
    's': 'backward',
    'a': 'left',
    'd': 'right',
    'space': 'up',
    'left shift': 'down',
    'right shift': 'down',
    'q': 'counter_clockwise',
    'e': 'clockwise',
    'left': 'counter_clockwise',
    'right': 'clockwise',
    'up': 'up',
    'down': 'down',
    'tab': lambda drone: drone.takeoff(),
    'backspace': lambda drone: drone.land(),
    # not implemented yet
    #'enter': lambda drone: drone.photo()
    #'r': lambda drone: drone.record_video()
    #'t': lambda drone: drone.finish_recording()
}

def handler(event, sender, data, **args):
    global prev_flight_data
    global video_player
    drone = sender
    if event is drone.EVENT_FLIGHT_DATA:
        if prev_flight_data != str(data):
            print(data)
            prev_flight_data = str(data)
    elif event is drone.EVENT_VIDEO_FRAME:
        if video_player is None:
            video_player = Popen(['mplayer', '-fps', '35', '-'], stdin=PIPE)
        try:
            video_player.stdin.write(data)
        except IOError as err:
            print(err)
            video_player = None
    else:
        print('event="%s" data=%s' % (event.getname(), str(data)))


def update(old, new, max_delta=0.3):
    if abs(old - new) <= max_delta:
        res = new
    else:
        res = 0.0
    return res


def main():
    pygame.init()
    pygame.display.init()
    pygame.display.set_mode((128,128))

    drone = tellopy.Tello()
    drone.connect()
    drone.start_video()
    drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)
    drone.subscribe(drone.EVENT_VIDEO_FRAME, handler)
    speed = 30
    throttle = 0.0
    yaw = 0.0
    pitch = 0.0
    roll = 0.0

    try:
        while 1:
            time.sleep(0.01)  # loop with pygame.event.get() is too mush tight w/o some sleep
            for e in pygame.event.get():
                # WASD for movement
                if e.type == pygame.locals.KEYDOWN:
                    print '+',pygame.key.name(e.key)
                    keyname = pygame.key.name(e.key)
                    if keyname == 'escape':
                        drone.quit()
                        exit(0)
                    if keyname in controls:
                        key_handler = controls[keyname]
                        if type(key_handler) == str:
                            getattr(drone, key_handler)(speed)
                        else:
                            key_handler(drone)

                elif e.type == pygame.locals.KEYUP:
                    print '-',pygame.key.name(e.key)
                    keyname = pygame.key.name(e.key)
                    if keyname in controls:
                        key_handler = controls[keyname]
                        if type(key_handler) == str:
                            getattr(drone, key_handler)(0)
                        else:
                            key_handler(drone)
    finally:
        print 'Shutting down connection to drone...'
        drone.quit()
        exit(1)

if __name__ == '__main__':
    main()
