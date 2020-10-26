"""
tellopy sample using keyboard and video player

Requires mencoder (part of mplayer) to record video.


Controls:
- tab to lift off
- WASD to move the drone
- space/shift to ascend/descent slowly
- Q/E to yaw slowly
- arrow keys to ascend, descend, or yaw quickly
- backspace to land, or P to palm-land
- enter to take a picture
- R to start recording video, R again to stop recording
  (video and photos will be saved to a timestamped file in ~/Pictures/)
- Z to toggle camera zoom state
  (zoomed-in widescreen or high FOV 4:3)
"""

import time
import tellopy
import pygame
import pygame.display
import pygame.key
import pygame.locals
import pygame.font
import pygame.surfarray
import os
import datetime
import threading
import traceback
import queue
import av
import cv2
import numpy
from subprocess import Popen, PIPE
from platform import system

prev_flight_data = None
flight_data = None
new_image = None
video_recorder = None
font = None
date_fmt = '%Y-%m-%d_%H%M%S'
status_queue = queue.Queue()


def get_file_path(file):
    if system() == 'Windows':
        return os.path.join(os.environ['USERPROFILE'] + "\\Pictures", file)
    else:
        return os.path.join(os.environ['HOME'] + "/Pictures", file)

def toggle_recording(drone, speed):
    global video_recorder
    global date_fmt
    if speed == 0:
        return

    if video_recorder:
        # already recording, so stop
        video_recorder.stdin.close()
        status_print('Video saved to %s' % video_recorder.video_filename)
        video_recorder = None
        return

    # start a new recording
    filename = 'tello-{}.mp4'.format (datetime.datetime.now().strftime(date_fmt))
    filename = get_file_path(filename)

    video_recorder = Popen([
        'mencoder', '-', '-vc', 'x264', '-fps', '30', '-ovc', 'copy',
        '-of', 'lavf', '-lavfopts', 'format=mp4',
        # '-ffourcc', 'avc1',
        # '-really-quiet',
        '-o', filename,
    ], stdin=PIPE)
    video_recorder.video_filename = filename
    status_print('Recording video to %s' % filename)

def take_picture(drone, speed):
    if speed == 0:
        return
    drone.take_picture()

def palm_land(drone, speed):
    if speed == 0:
        return
    drone.palm_land()

def toggle_zoom(drone, speed):
    # In "video" mode the drone sends 1280x720 frames.
    # In "photo" mode it sends 2592x1936 (952x720) frames.
    # The video will always be centered in the window.
    # In photo mode, if we keep the window at 1280x720 that gives us ~160px on
    # each side for status information, which is ample.
    # Video mode is harder because then we need to abandon the 16:9 display size
    # if we want to put the HUD next to the video.
    if speed == 0:
        return
    drone.set_video_mode(not drone.zoom)
    pygame.display.get_surface().fill((0,0,0))
    pygame.display.flip()

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
    # arrow keys for fast turns and altitude adjustments
    'left': lambda drone, speed: drone.counter_clockwise(speed*2),
    'right': lambda drone, speed: drone.clockwise(speed*2),
    'up': lambda drone, speed: drone.up(speed*2),
    'down': lambda drone, speed: drone.down(speed*2),
    'tab': lambda drone, speed: drone.takeoff(),
    'backspace': lambda drone, speed: drone.land(),
    'p': palm_land,
    'r': toggle_recording,
    'z': toggle_zoom,
    'enter': take_picture,
    'return': take_picture,
}

class FlightDataDisplay(object):
    # previous flight data value and surface to overlay
    _value = None
    _surface = None
    # function (drone, data) => new value
    # default is lambda drone,data: getattr(data, self._key)
    _update = None
    def __init__(self, key, format, colour=(255,255,255), update=None):
        self._key = key
        self._format = format
        self._colour = colour

        if update:
            self._update = update
        else:
            self._update = lambda drone,data: getattr(data, self._key)

    def update(self, drone, data):
        new_value = self._update(drone, data)
        if self._value != new_value:
            self._value = new_value
            self._surface = font.render(self._format % (new_value,), True, self._colour)
        return self._surface

def flight_data_mode(drone, *args):
    return (drone.zoom and "VID" or "PIC")

def flight_data_recording(*args):
    return (video_recorder and "REC 00:00" or "")  # TODO: duration of recording

def update_hud(hud, drone, flight_data):
    (w,h) = (158,0) # width available on side of screen in 4:3 mode
    blits = []
    for element in hud:
        surface = element.update(drone, flight_data)
        if surface is None:
            continue
        blits += [(surface, (0, h))]
        w = max(w, surface.get_width())
        h += surface.get_height()
    h += 64  # add some padding
    overlay = pygame.Surface((w, h), pygame.SRCALPHA)
    overlay.fill((0,0,0))
    for blit in blits:
        overlay.blit(*blit)
    pygame.display.get_surface().blit(overlay, (0,0))
    # pygame.display.update() is called once per frame in main()'s loop,
    # after the video frame and the HUD are both blitted -- not here,
    # since this may be called from a thread other than the main one
    # (Cocoa/AppKit requires window updates to happen on the main thread).

def status_print(text):
    # status_print() is called from several threads (e.g. the video
    # recorder's IOError handler, EVENT_FILE_RECEIVED); only the main
    # thread may touch the window (pygame.display.set_caption()), so
    # queue the text and let main()'s loop apply it.
    status_queue.put(text)

hud = [
    FlightDataDisplay('height', 'ALT %3d'),
    FlightDataDisplay('ground_speed', 'SPD %3d'),
    FlightDataDisplay('battery_percentage', 'BAT %3d%%'),
    FlightDataDisplay('wifi_strength', 'NET %3d%%'),
    FlightDataDisplay(None, 'CAM %s', update=flight_data_mode),
    FlightDataDisplay(None, '%s', colour=(255, 0, 0), update=flight_data_recording),
]

def flightDataHandler(event, sender, data):
    # Just record the latest sample; main()'s loop decides whether it
    # changed and redraws the HUD from there (the only thread allowed to
    # touch the pygame window).
    global flight_data
    flight_data = data

def recv_thread(drone):
    # Decode the video stream and hand the latest frame to main()'s loop
    # via new_image, the same way joystick_and_video.py/video_effect.py
    # do -- this thread never touches pygame/the window itself.
    global new_image
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

def videoRecorderHandler(event, sender, data):
    # Only feeds the mencoder recording pipe now; display no longer goes
    # through mplayer (see recv_thread()). EVENT_VIDEO_FRAME and the
    # EVENT_VIDEO_DATA that get_video_stream() consumes are published
    # independently per packet, so the two don't compete for data.
    global video_recorder
    try:
        if video_recorder:
            video_recorder.stdin.write(data)
    except IOError as err:
        status_print(str(err))
        video_recorder = None

def handleFileReceived(event, sender, data):
    global date_fmt
    # Create a file in ~/Pictures/ to receive image data from the drone.
    file = 'tello-{}.jpeg'.format(datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S'))

    path = get_file_path(file)
    with open(path, 'wb') as fd:
        fd.write(data)
    status_print('Saved photo to %s' % path)

def main():
    pygame.init()
    pygame.display.init()
    pygame.display.set_mode((1280, 720))
    pygame.font.init()

    global font
    font = pygame.font.SysFont("dejavusansmono", 32)

    drone = tellopy.Tello()
    drone.connect()
    drone.subscribe(drone.EVENT_FLIGHT_DATA, flightDataHandler)
    drone.subscribe(drone.EVENT_VIDEO_FRAME, videoRecorderHandler)
    drone.subscribe(drone.EVENT_FILE_RECEIVED, handleFileReceived)
    threading.Thread(target=recv_thread, args=[drone]).start()
    speed = 30

    current_image = None
    prev_flight_data_text = None

    try:
        while 1:
            time.sleep(0.01)  # loop with pygame.event.get() is too mush tight w/o some sleep

            # Apply any status text queued from other threads (see status_print()).
            try:
                while True:
                    pygame.display.set_caption(status_queue.get_nowait())
            except queue.Empty:
                pass

            for e in pygame.event.get():
                # WASD for movement
                if e.type == pygame.locals.KEYDOWN:
                    print('+' + pygame.key.name(e.key))
                    keyname = pygame.key.name(e.key)
                    if keyname == 'escape':
                        drone.quit()
                        exit(0)
                    if keyname in controls:
                        key_handler = controls[keyname]
                        if type(key_handler) == str:
                            getattr(drone, key_handler)(speed)
                        else:
                            key_handler(drone, speed)

                elif e.type == pygame.locals.KEYUP:
                    print('-' + pygame.key.name(e.key))
                    keyname = pygame.key.name(e.key)
                    if keyname in controls:
                        key_handler = controls[keyname]
                        if type(key_handler) == str:
                            getattr(drone, key_handler)(0)
                        else:
                            key_handler(drone, 0)

            # Redraw (video frame + HUD, in that order so the HUD stays on
            # top) only when something to show has actually changed, and
            # only here, on the main thread.
            redraw = current_image is not new_image
            current_image = new_image
            text = str(flight_data) if flight_data is not None else None
            redraw = redraw or text != prev_flight_data_text
            prev_flight_data_text = text

            if redraw and current_image is not None:
                frame_rgb = cv2.cvtColor(current_image, cv2.COLOR_BGR2RGB)
                frame_surface = pygame.surfarray.make_surface(frame_rgb.swapaxes(0, 1))
                # Centre the frame: in photo mode it is narrower (952x720)
                # than the 1280x720 window, leaving room on both sides for
                # the HUD, which is anchored at the left edge.
                x = (1280 - frame_surface.get_width()) // 2
                pygame.display.get_surface().blit(frame_surface, (x, 0))
                if flight_data is not None:
                    update_hud(hud, drone, flight_data)
                pygame.display.update()
    except Exception as e:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(str(e))
    finally:
        print('Shutting down connection to drone...')
        if video_recorder:
            toggle_recording(drone, 1)
        drone.quit()
        exit(1)

if __name__ == '__main__':
    main()
