"""
tellopy sample using joystick and video palyer

 - you can use PS3/PS4/XONE joystick to controll DJI Tello with tellopy module
 - you must install mplayer to replay the video
 - Xbox One Controllers were only tested on Mac OS with the 360Controller Driver.
    get it here -> https://github.com/360Controller/360Controller'''
"""

import time
import sys
import tellopy
import pygame
import pygame.locals
from subprocess import Popen, PIPE
import threading
import av
import cv2.cv2 as cv2  # for avoidance of pylint error
import numpy
import time
import traceback


class JoystickPS3:
    # d-pad
    UP = 4  # UP
    DOWN = 6  # DOWN
    ROTATE_LEFT = 7  # LEFT
    ROTATE_RIGHT = 5  # RIGHT

    # bumper triggers
    TAKEOFF = 11  # R1
    LAND = 10  # L1
    # UNUSED = 9 #R2
    # UNUSED = 8 #L2

    # buttons
    FORWARD = 12  # TRIANGLE
    BACKWARD = 14  # CROSS
    LEFT = 15  # SQUARE
    RIGHT = 13  # CIRCLE

    # axis
    LEFT_X = 0
    LEFT_Y = 1
    RIGHT_X = 2
    RIGHT_Y = 3
    LEFT_X_REVERSE = 1.0
    LEFT_Y_REVERSE = -1.0
    RIGHT_X_REVERSE = 1.0
    RIGHT_Y_REVERSE = -1.0
    DEADZONE = 0.1


class JoystickPS4:
    # d-pad
    UP = -1  # UP
    DOWN = -1  # DOWN
    ROTATE_LEFT = -1  # LEFT
    ROTATE_RIGHT = -1  # RIGHT

    # bumper triggers
    TAKEOFF = 5  # R1
    LAND = 4  # L1
    # UNUSED = 7 #R2
    # UNUSED = 6 #L2

    # buttons
    FORWARD = 3  # TRIANGLE
    BACKWARD = 1  # CROSS
    LEFT = 0  # SQUARE
    RIGHT = 2  # CIRCLE

    # axis
    LEFT_X = 0
    LEFT_Y = 1
    RIGHT_X = 2
    RIGHT_Y = 3
    LEFT_X_REVERSE = 1.0
    LEFT_Y_REVERSE = -1.0
    RIGHT_X_REVERSE = 1.0
    RIGHT_Y_REVERSE = -1.0
    DEADZONE = 0.08


class JoystickPS4ALT:
    # d-pad
    UP = -1  # UP
    DOWN = -1  # DOWN
    ROTATE_LEFT = -1  # LEFT
    ROTATE_RIGHT = -1  # RIGHT

    # bumper triggers
    TAKEOFF = 5  # R1
    LAND = 4  # L1
    # UNUSED = 7 #R2
    # UNUSED = 6 #L2

    # buttons
    FORWARD = 3  # TRIANGLE
    BACKWARD = 1  # CROSS
    LEFT = 0  # SQUARE
    RIGHT = 2  # CIRCLE

    # axis
    LEFT_X = 0
    LEFT_Y = 1
    RIGHT_X = 3
    RIGHT_Y = 4
    LEFT_X_REVERSE = 1.0
    LEFT_Y_REVERSE = -1.0
    RIGHT_X_REVERSE = 1.0
    RIGHT_Y_REVERSE = -1.0
    DEADZONE = 0.08

class JoystickF310:
    # d-pad
    UP = -1  # UP
    DOWN = -1  # DOWN
    ROTATE_LEFT = -1  # LEFT
    ROTATE_RIGHT = -1  # RIGHT

    # bumper triggers
    TAKEOFF = 5  # R1
    LAND = 4  # L1
    # UNUSED = 7 #R2
    # UNUSED = 6 #L2

    # buttons
    FORWARD = 3  # Y
    BACKWARD = 0  # B
    LEFT = 2  # X
    RIGHT = 1  # A

    # axis
    LEFT_X = 0
    LEFT_Y = 1
    RIGHT_X = 3
    RIGHT_Y = 4
    LEFT_X_REVERSE = 1.0
    LEFT_Y_REVERSE = -1.0
    RIGHT_X_REVERSE = 1.0
    RIGHT_Y_REVERSE = -1.0
    DEADZONE = 0.08

class JoystickXONE:
    # d-pad
    UP = 0  # UP
    DOWN = 1  # DOWN
    ROTATE_LEFT = 2  # LEFT
    ROTATE_RIGHT = 3  # RIGHT

    # bumper triggers
    TAKEOFF = 9  # RB
    LAND = 8  # LB
    # UNUSED = 7 #RT
    # UNUSED = 6 #LT

    # buttons
    FORWARD = 14  # Y
    BACKWARD = 11  # A
    LEFT = 13  # X
    RIGHT = 12  # B

    # axis
    LEFT_X = 0
    LEFT_Y = 1
    RIGHT_X = 2
    RIGHT_Y = 3
    LEFT_X_REVERSE = 1.0
    LEFT_Y_REVERSE = -1.0
    RIGHT_X_REVERSE = 1.0
    RIGHT_Y_REVERSE = -1.0
    DEADZONE = 0.09


class JoystickTARANIS:
    # d-pad
    UP = -1  # UP
    DOWN = -1  # DOWN
    ROTATE_LEFT = -1  # LEFT
    ROTATE_RIGHT = -1  # RIGHT

    # bumper triggers
    TAKEOFF = 12  # left switch
    LAND = 12  # left switch
    # UNUSED = 7 #RT
    # UNUSED = 6 #LT

    # buttons
    FORWARD = -1
    BACKWARD = -1
    LEFT = -1
    RIGHT = -1

    # axis
    LEFT_X = 3
    LEFT_Y = 0
    RIGHT_X = 1
    RIGHT_Y = 2
    LEFT_X_REVERSE = 1.0
    LEFT_Y_REVERSE = 1.0
    RIGHT_X_REVERSE = 1.0
    RIGHT_Y_REVERSE = 1.0
    DEADZONE = 0.01


prev_flight_data = None
run_recv_thread = True
new_image = None
flight_data = None
log_data = None
buttons = None
speed = 100
throttle = 0.0
yaw = 0.0
pitch = 0.0
roll = 0.0

def handler(event, sender, data, **args):
    global prev_flight_data
    global flight_data
    global log_data
    drone = sender
    if event is drone.EVENT_FLIGHT_DATA:
        if prev_flight_data != str(data):
            print(data)
            prev_flight_data = str(data)
        flight_data = data
    elif event is drone.EVENT_LOG_DATA:
        log_data = data
    else:
        print('event="%s" data=%s' % (event.getname(), str(data)))


def update(old, new, max_delta=0.3):
    if abs(old - new) <= max_delta:
        res = new
    else:
        res = 0.0
    return res


def handle_input_event(drone, e):
    global speed
    global throttle
    global yaw
    global pitch
    global roll
    if e.type == pygame.locals.JOYAXISMOTION:
        # ignore small input values (Deadzone)
        if -buttons.DEADZONE <= e.value and e.value <= buttons.DEADZONE:
            e.value = 0.0
        if e.axis == buttons.LEFT_Y:
            throttle = update(throttle, e.value * buttons.LEFT_Y_REVERSE)
            drone.set_throttle(throttle)
        if e.axis == buttons.LEFT_X:
            yaw = update(yaw, e.value * buttons.LEFT_X_REVERSE)
            drone.set_yaw(yaw)
        if e.axis == buttons.RIGHT_Y:
            pitch = update(pitch, e.value *
                           buttons.RIGHT_Y_REVERSE)
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
        if e.button == buttons.LAND:
            drone.land()
        elif e.button == buttons.UP:
            drone.up(speed)
        elif e.button == buttons.DOWN:
            drone.down(speed)
        elif e.button == buttons.ROTATE_RIGHT:
            drone.clockwise(speed)
        elif e.button == buttons.ROTATE_LEFT:
            drone.counter_clockwise(speed)
        elif e.button == buttons.FORWARD:
            drone.forward(speed)
        elif e.button == buttons.BACKWARD:
            drone.backward(speed)
        elif e.button == buttons.RIGHT:
            drone.right(speed)
        elif e.button == buttons.LEFT:
            drone.left(speed)
    elif e.type == pygame.locals.JOYBUTTONUP:
        if e.button == buttons.TAKEOFF:
            if throttle != 0.0:
                print('###')
                print('### throttle != 0.0 (This may hinder the drone from taking off)')
                print('###')
            drone.takeoff()
        elif e.button == buttons.UP:
            drone.up(0)
        elif e.button == buttons.DOWN:
            drone.down(0)
        elif e.button == buttons.ROTATE_RIGHT:
            drone.clockwise(0)
        elif e.button == buttons.ROTATE_LEFT:
            drone.counter_clockwise(0)
        elif e.button == buttons.FORWARD:
            drone.forward(0)
        elif e.button == buttons.BACKWARD:
            drone.backward(0)
        elif e.button == buttons.RIGHT:
            drone.right(0)
        elif e.button == buttons.LEFT:
            drone.left(0)

def draw_text(image, text, row):
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        font_size = 24
        font_color = (255,255,255)
        bg_color = (0,0,0)
        d = 2
        height, width = image.shape[:2]
        left_mergin = 10
        if row < 0:
            pos =  (left_mergin, height + font_size * row + 1)
        else:
            pos =  (left_mergin, font_size * (row + 1))
        cv2.putText(image, text, pos, font, font_scale, bg_color, 6)
        cv2.putText(image, text, pos, font, font_scale, font_color, 1)

def recv_thread(drone):
    global run_recv_thread
    global new_image
    global flight_data
    global log_data

    print('start recv_thread()')
    try:
        container = av.open(drone.get_video_stream())
        # skip first 300 frames
        frame_skip = 300
        while True:
            for frame in container.decode(video=0):
                if 0 < frame_skip:
                    frame_skip = frame_skip - 1
                    continue
                start_time = time.time()
                image = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)

                if flight_data:
                    draw_text(image, 'TelloPy: joystick_and_video ' + str(flight_data), 0)
                if log_data:
                    draw_text(image, 'MVO: ' + str(log_data.mvo), -3)
                    draw_text(image, ('IMU: ' + str(log_data.imu))[0:52], -2)
                    draw_text(image, '     ' + ('IMU: ' + str(log_data.imu))[52:], -1)
                new_image = image
                if frame.time_base < 1.0/60:
                    time_base = 1.0/60
                else:
                    time_base = frame.time_base
                frame_skip = int((time.time() - start_time)/time_base)
    except Exception as ex:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(ex)

def main():
    global buttons
    global run_recv_thread
    global new_image
    pygame.init()
    pygame.joystick.init()
    current_image = None

    try:
        js = pygame.joystick.Joystick(0)
        js.init()
        js_name = js.get_name()
        print('Joystick name: ' + js_name)
        if js_name in ('Wireless Controller', 'Sony Computer Entertainment Wireless Controller'):
            buttons = JoystickPS4
        elif js_name == 'Sony Interactive Entertainment Wireless Controller':
            buttons = JoystickPS4ALT
        elif js_name in ('PLAYSTATION(R)3 Controller', 'Sony PLAYSTATION(R)3 Controller'):
            buttons = JoystickPS3
        elif js_name in ('Logitech Gamepad F310'):
            buttons = JoystickF310
        elif js_name == 'Xbox One Wired Controller':
            buttons = JoystickXONE
        elif js_name == 'FrSky Taranis Joystick':
            buttons = JoystickTARANIS
    except pygame.error:
        pass

    if buttons is None:
        print('no supported joystick found')
        return

    drone = tellopy.Tello()
    drone.connect()
    drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)
    drone.subscribe(drone.EVENT_LOG_DATA, handler)
    threading.Thread(target=recv_thread, args=[drone]).start()

    try:
        while 1:
            # loop with pygame.event.get() is too much tight w/o some sleep
            time.sleep(0.01)
            for e in pygame.event.get():
                handle_input_event(drone, e)
            if current_image is not new_image:
                cv2.imshow('Tello', new_image)
                current_image = new_image
                cv2.waitKey(1)
    except KeyboardInterrupt as e:
        print(e)
    except Exception as e:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(e)

    run_recv_thread = False
    cv2.destroyAllWindows()
    drone.quit()
    exit(1)


if __name__ == '__main__':
    main()
