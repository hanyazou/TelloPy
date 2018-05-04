"""
tellopy sample using joystick and video palyer

 - you can use PS3/PS4 joystick to controll DJI Tello with tellopy module
 - you must install mplayer to replay the video
"""

import time
import sys
import tellopy
import pygame
import pygame.locals
from subprocess import Popen, PIPE


class JoystickPS3:
    # buttons
    UP = 4
    RIGHT = 5
    DOWN = 6
    LEFT = 7
    L2 = 8
    R2 = 9
    L1 = 10
    R1 = 11
    TRIANGLE = 12
    CIRCLE = 13
    CROSS = 14
    SQUARE = 15
    # axis
    LEFT_X = 0
    LEFT_Y = 1
    RIGHT_X = 2
    RIGHT_Y = 3
    LEFT_X_REVERSE = 1.0
    LEFT_Y_REVERSE = -1.0
    RIGHT_X_REVERSE = 1.0
    RIGHT_Y_REVERSE = -1.0
    BACKSLASH = 0.1


class JoystickPS4:
    # buttons
    UP = -1
    RIGHT = -1
    DOWN = -1
    LEFT = -1
    L2 = 6
    R2 = 7
    L1 = 4
    R1 = 5
    TRIANGLE = 3
    CIRCLE = 2
    CROSS = 1
    SQUARE = 0
    # axis
    LEFT_X = 0
    LEFT_Y = 1
    RIGHT_X = 2
    RIGHT_Y = 3
    LEFT_X_REVERSE = 1.0
    LEFT_Y_REVERSE = -1.0
    RIGHT_X_REVERSE = 1.0
    RIGHT_Y_REVERSE = -1.0
    BACKSLASH = 0.08

prev_flight_data = None
video_player = None


def handler(event, sender, data, **args):
    global prev_flight_data
    global video_player
    drone = sender
    if event is drone.CONNECTED_EVENT:
        print('connected')
        drone.start_video()
        drone.set_exposure(0)
        drone.set_video_encoder_rate(4)
    elif event is drone.FLIGHT_EVENT:
        if prev_flight_data != str(data):
            print(data)
            prev_flight_data = str(data)
    elif event is drone.VIDEO_FRAME_EVENT:
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
    pygame.joystick.init()
    buttons = None
    try:
        js = pygame.joystick.Joystick(0)
        js.init()
        js_name = js.get_name()
        print('Joystick name: ' + js_name)
        if js_name in ('Wireless Controller', 'Sony Computer Entertainment Wireless Controller'):
            buttons = JoystickPS4
        elif js_name == ('PLAYSTATION(R)3 Controller', 'Sony PLAYSTATION(R)3 Controller'):
            buttons = JoystickPS3
    except pygame.error:
        pass

    if buttons is None:
        print('no supported joystick found')
        return

    drone = tellopy.Tello()
    drone.connect()
    drone.subscribe(drone.CONNECTED_EVENT, handler)
    drone.subscribe(drone.FLIGHT_EVENT, handler)
    drone.subscribe(drone.VIDEO_FRAME_EVENT, handler)
    speed = 30
    throttle = 0.0
    yaw = 0.0
    pitch = 0.0
    roll = 0.0

    try:
        while 1:
            time.sleep(0.01)  # loop with pygame.event.get() is too mush tight w/o some sleep
            for e in pygame.event.get():
                if e.type == pygame.locals.JOYAXISMOTION:
                    # ignore small backslash
                    if -buttons.BACKSLASH <= e.value and e.value <= buttons.BACKSLASH:
                        e.value = 0.0
                    if e.axis == buttons.LEFT_Y:
                        throttle = update(throttle, e.value * buttons.LEFT_Y_REVERSE)
                        drone.set_throttle(throttle)
                    if e.axis == buttons.LEFT_X:
                        yaw = update(yaw, e.value * buttons.LEFT_X_REVERSE)
                        drone.set_yaw(yaw)
                    if e.axis == buttons.RIGHT_Y:
                        pitch = update(pitch, e.value * buttons.RIGHT_Y_REVERSE)
                        drone.set_pitch(pitch)
                    if e.axis == buttons.RIGHT_X:
                        roll = update(roll, e.value * buttons.RIGHT_X_REVERSE)
                        drone.set_roll(roll)

                elif e.type == pygame.locals.JOYHATMOTION:
                    if e.value[0] < 0:
                        drone.counter_clockwise(speed)
                    if e.value[0] == 0:
                        drone.clockwise(0)
                    if e.value[0] > 0:
                        drone.clockwise(speed)
                    if e.value[1] < 0:
                        drone.down(speed)
                    if e.value[1] == 0:
                        drone.up(0)
                    if e.value[1] > 0:
                        drone.up(speed)
                elif e.type == pygame.locals.JOYBUTTONDOWN:
                    if e.button == buttons.L1:
                        drone.land()
                    elif e.button == buttons.UP:
                        drone.up(speed)
                    elif e.button == buttons.DOWN:
                        drone.down(speed)
                    elif e.button == buttons.RIGHT:
                        drone.clockwise(speed)
                    elif e.button == buttons.LEFT:
                        drone.counter_clockwise(speed)
                    elif e.button == buttons.TRIANGLE:
                        drone.forward(speed)
                    elif e.button == buttons.CROSS:
                        drone.backward(speed)
                    elif e.button == buttons.CIRCLE:
                        drone.right(speed)
                    elif e.button == buttons.SQUARE:
                        drone.left(speed)
                elif e.type == pygame.locals.JOYBUTTONUP:
                    if e.button == buttons.R1:
                        drone.takeoff()
                    elif e.button == buttons.UP:
                        drone.up(0)
                    elif e.button == buttons.DOWN:
                        drone.down(0)
                    elif e.button == buttons.RIGHT:
                        drone.clockwise(0)
                    elif e.button == buttons.LEFT:
                        drone.counter_clockwise(0)
                    elif e.button == buttons.TRIANGLE:
                        drone.forward(0)
                    elif e.button == buttons.CROSS:
                        drone.backward(0)
                    elif e.button == buttons.CIRCLE:
                        drone.right(0)
                    elif e.button == buttons.SQUARE:
                        drone.left(0)
    except KeyboardInterrupt as e:
        print(e)
    except Exception as e:
        print(e)

    drone.quit()
    exit(1)

if __name__ == '__main__':
    main()
