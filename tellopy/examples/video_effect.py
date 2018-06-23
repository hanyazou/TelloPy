import sys
import traceback
import tellopy
import av
import cv2.cv2 as cv2  # for avoidance of pylint error
import numpy
import threading
import time

frame = None
run_recv_thread = True

def recv_thread():
    global frame
    global run_recv_thread

    print('start recv_thread()')
    drone = tellopy.Tello()

    try:
        drone.connect()
        drone.wait_for_connection(60.0)

        container = av.open(drone.get_video_stream())
        frame_count = 0
        while run_recv_thread:
            for f in container.decode(video=0):
                frame_count = frame_count + 1
                # skip first 300 frames
                if frame_count < 300:
                    continue
                frame = f
            time.sleep(0.01)
    except Exception as ex:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(ex)
    finally:
        drone.quit()

def main():
    global frame

    try:
        threading.Thread(target=recv_thread).start()

        while True:
            if frame is None:
                time.sleep(0.01)
            else:
                image = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)
                cv2.imshow('Original', image)
                cv2.imshow('Canny', cv2.Canny(image, 100, 200))
                cv2.waitKey(1)
                # long deley
                time.sleep(0.5)
                image = None

    except Exception as ex:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(ex)
    finally:
        run_recv_thread = False
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
