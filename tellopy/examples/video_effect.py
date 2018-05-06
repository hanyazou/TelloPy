import tellopy
import av
import cv2.cv2 as cv2  # for avoidance of pylint error
import numpy


def main():
    drone = tellopy.Tello()

    try:
        drone.connect()
        drone.wait_for_connection(60.0)

        container = av.open(drone.get_video_stream())
        while True:
            frame = container.decode(video=0).next()

            image = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)
            cv2.imshow('Original', image)
            cv2.imshow('Canny', cv2.Canny(image, 100, 200))
            cv2.waitKey(1)

    except Exception as ex:
        print(ex)
    finally:
        drone.quit()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
