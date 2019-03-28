import threading
import socket
import time
import datetime
import struct
import sys
import os

from . import crc
from . import logger
from . import event
from . import state
from . import error
from . import video_stream
from . utils import *
from . protocol import *
from . import dispatcher

log = logger.Logger('Tello')


class Tello(object):
    EVENT_CONNECTED = event.Event('connected')
    EVENT_WIFI = event.Event('wifi')
    EVENT_LIGHT = event.Event('light')
    EVENT_FLIGHT_DATA = event.Event('fligt_data')
    EVENT_LOG_HEADER = event.Event('log_header')
    EVENT_LOG = EVENT_LOG_HEADER
    EVENT_LOG_RAWDATA = event.Event('log_rawdata')
    EVENT_LOG_DATA = event.Event('log_data')
    EVENT_LOG_CONFIG = event.Event('log_config')
    EVENT_TIME = event.Event('time')
    EVENT_VIDEO_FRAME = event.Event('video frame')
    EVENT_VIDEO_DATA = event.Event('video data')
    EVENT_DISCONNECTED = event.Event('disconnected')
    EVENT_FILE_RECEIVED = event.Event('file received')
    # internal events
    __EVENT_CONN_REQ = event.Event('conn_req')
    __EVENT_CONN_ACK = event.Event('conn_ack')
    __EVENT_TIMEOUT = event.Event('timeout')
    __EVENT_QUIT_REQ = event.Event('quit_req')

    # for backward comaptibility
    CONNECTED_EVENT = EVENT_CONNECTED
    WIFI_EVENT = EVENT_WIFI
    LIGHT_EVENT = EVENT_LIGHT
    FLIGHT_EVENT = EVENT_FLIGHT_DATA
    LOG_EVENT = EVENT_LOG
    TIME_EVENT = EVENT_TIME
    VIDEO_FRAME_EVENT = EVENT_VIDEO_FRAME

    STATE_DISCONNECTED = state.State('disconnected')
    STATE_CONNECTING = state.State('connecting')
    STATE_CONNECTED = state.State('connected')
    STATE_QUIT = state.State('quit')

    LOG_ERROR = logger.LOG_ERROR
    LOG_WARN = logger.LOG_WARN
    LOG_INFO = logger.LOG_INFO
    LOG_DEBUG = logger.LOG_DEBUG
    LOG_ALL = logger.LOG_ALL

    def __init__(self, port=9000):
        self.tello_addr = ('192.168.10.1', 8889)
        self.debug = False
        self.pkt_seq_num = 0x01e4
        self.port = port
        self.udpsize = 2000
        self.left_x = 0.0
        self.left_y = 0.0
        self.right_x = 0.0
        self.right_y = 0.0
        self.sock = None
        self.state = self.STATE_DISCONNECTED
        self.lock = threading.Lock()
        self.connected = threading.Event()
        self.video_enabled = False
        self.prev_video_data_time = None
        self.video_data_size = 0
        self.video_data_loss = 0
        self.log = log
        self.exposure = 0
        self.video_encoder_rate = 4
        self.video_stream = None
        self.wifi_strength = 0
        self.log_data = LogData(log)
        self.log_data_file = None
        self.log_data_header_recorded = False

        # video zoom state
        self.zoom = False

        # File recieve state.
        self.file_recv = {}  # Map filenum -> protocol.DownloadedFile

        # Create a UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', self.port))
        self.sock.settimeout(2.0)

        dispatcher.connect(self.__state_machine, dispatcher.signal.All)
        threading.Thread(target=self.__recv_thread).start()
        threading.Thread(target=self.__video_thread).start()

    def set_loglevel(self, level):
        """
        Set_loglevel controls the output messages. Valid levels are
        LOG_ERROR, LOG_WARN, LOG_INFO, LOG_DEBUG and LOG_ALL.
        """
        log.set_level(level)

    def get_video_stream(self):
        """
        Get_video_stream is used to prepare buffer object which receive video data from the drone.
        """
        newly_created = False
        self.lock.acquire()
        log.info('get video stream')
        try:
            if self.video_stream is None:
                self.video_stream = video_stream.VideoStream(self)
                newly_created = True
            res = self.video_stream
        finally:
            self.lock.release()
        if newly_created:
            self.__send_exposure()
            self.__send_video_encoder_rate()
            self.start_video()

        return res

    def connect(self):
        """Connect is used to send the initial connection request to the drone."""
        self.__publish(event=self.__EVENT_CONN_REQ)

    def wait_for_connection(self, timeout=None):
        """Wait_for_connection will block until the connection is established."""
        if not self.connected.wait(timeout):
            raise error.TelloError('timeout')

    def __send_conn_req(self):
        port = 9617
        port0 = (int(port/1000) % 10) << 4 | (int(port/100) % 10)
        port1 = (int(port/10) % 10) << 4 | (int(port/1) % 10)
        buf = 'conn_req:%c%c' % (chr(port0), chr(port1))
        log.info('send connection request (cmd="%s%02x%02x")' % (str(buf[:-2]), port0, port1))
        return self.send_packet(Packet(buf))

    def subscribe(self, signal, handler):
        """Subscribe a event such as EVENT_CONNECTED, EVENT_FLIGHT_DATA, EVENT_VIDEO_FRAME and so on."""
        dispatcher.connect(handler, signal)

    def __publish(self, event, data=None, **args):
        args.update({'data': data})
        if 'signal' in args:
            del args['signal']
        if 'sender' in args:
            del args['sender']
        log.debug('publish signal=%s, args=%s' % (event, args))
        dispatcher.send(event, sender=self, **args)

    def takeoff(self):
        """Takeoff tells the drones to liftoff and start flying."""
        log.info('set altitude limit 30m')
        pkt = Packet(SET_ALT_LIMIT_CMD)
        pkt.add_byte(0x1e)  # 30m
        pkt.add_byte(0x00)
        self.send_packet(pkt)
        log.info('takeoff (cmd=0x%02x seq=0x%04x)' % (TAKEOFF_CMD, self.pkt_seq_num))
        pkt = Packet(TAKEOFF_CMD)
        pkt.fixup()
        return self.send_packet(pkt)

    def throw_and_go(self):
        """Throw_and_go starts a throw and go sequence"""
        log.info('throw_and_go (cmd=0x%02x seq=0x%04x)' % (THROW_AND_GO_CMD, self.pkt_seq_num))
        pkt = Packet(THROW_AND_GO_CMD, 0x48)
        pkt.add_byte(0x00)
        pkt.fixup()
        return self.send_packet(pkt)

    def land(self):
        """Land tells the drone to come in for landing."""
        log.info('land (cmd=0x%02x seq=0x%04x)' % (LAND_CMD, self.pkt_seq_num))
        pkt = Packet(LAND_CMD)
        pkt.add_byte(0x00)
        pkt.fixup()
        return self.send_packet(pkt)

    def palm_land(self):
        """Tells the drone to wait for a hand underneath it and then land."""
        log.info('palmland (cmd=0x%02x seq=0x%04x)' % (PALM_LAND_CMD, self.pkt_seq_num))
        pkt = Packet(PALM_LAND_CMD)
        pkt.add_byte(0x00)
        pkt.fixup()
        return self.send_packet(pkt)

    def quit(self):
        """Quit stops the internal threads."""
        log.info('quit')
        self.__publish(event=self.__EVENT_QUIT_REQ)

    def __send_time_command(self):
        log.info('send_time (cmd=0x%02x seq=0x%04x)' % (TIME_CMD, self.pkt_seq_num))
        pkt = Packet(TIME_CMD, 0x50)
        pkt.add_byte(0)
        pkt.add_time()
        pkt.fixup()
        return self.send_packet(pkt)

    def __send_start_video(self):
        pkt = Packet(VIDEO_START_CMD, 0x60)
        pkt.fixup()
        return self.send_packet(pkt)

    def __send_video_mode(self, mode):
        pkt = Packet(VIDEO_MODE_CMD)
        pkt.add_byte(mode)
        pkt.fixup()
        return self.send_packet(pkt)

    def set_video_mode(self, zoom=False):
        """Tell the drone whether to capture 960x720 4:3 video, or 1280x720 16:9 zoomed video.
        4:3 has a wider field of view (both vertically and horizontally), 16:9 is crisper."""
        log.info('set video mode zoom=%s (cmd=0x%02x seq=0x%04x)' % (
            zoom, VIDEO_START_CMD, self.pkt_seq_num))
        self.zoom = zoom
        return self.__send_video_mode(int(zoom))

    def start_video(self):
        """Start_video tells the drone to send start info (SPS/PPS) for video stream."""
        log.info('start video (cmd=0x%02x seq=0x%04x)' % (VIDEO_START_CMD, self.pkt_seq_num))
        self.video_enabled = True
        self.__send_exposure()
        self.__send_video_encoder_rate()
        return self.__send_start_video()

    def set_exposure(self, level):
        """Set_exposure sets the drone camera exposure level. Valid levels are 0, 1, and 2."""
        if level < 0 or 2 < level:
            raise error.TelloError('Invalid exposure level')
        log.info('set exposure (cmd=0x%02x seq=0x%04x)' % (EXPOSURE_CMD, self.pkt_seq_num))
        self.exposure = level
        return self.__send_exposure()

    def __send_exposure(self):
        pkt = Packet(EXPOSURE_CMD, 0x48)
        pkt.add_byte(self.exposure)
        pkt.fixup()
        return self.send_packet(pkt)

    def set_video_encoder_rate(self, rate):
        """Set_video_encoder_rate sets the drone video encoder rate."""
        log.info('set video encoder rate (cmd=0x%02x seq=%04x)' %
                 (VIDEO_ENCODER_RATE_CMD, self.pkt_seq_num))
        self.video_encoder_rate = rate
        return self.__send_video_encoder_rate()

    def __send_video_encoder_rate(self):
        pkt = Packet(VIDEO_ENCODER_RATE_CMD, 0x68)
        pkt.add_byte(self.video_encoder_rate)
        pkt.fixup()
        return self.send_packet(pkt)

    def take_picture(self):
        log.info('take picture')
        return self.send_packet_data(TAKE_PICTURE_COMMAND, type=0x68)

    def up(self, val):
        """Up tells the drone to ascend. Pass in an int from 0-100."""
        log.info('up(val=%d)' % val)
        self.left_y = val / 100.0

    def down(self, val):
        """Down tells the drone to descend. Pass in an int from 0-100."""
        log.info('down(val=%d)' % val)
        self.left_y = val / 100.0 * -1

    def forward(self, val):
        """Forward tells the drone to go forward. Pass in an int from 0-100."""
        log.info('forward(val=%d)' % val)
        self.right_y = val / 100.0

    def backward(self, val):
        """Backward tells the drone to go in reverse. Pass in an int from 0-100."""
        log.info('backward(val=%d)' % val)
        self.right_y = val / 100.0 * -1

    def right(self, val):
        """Right tells the drone to go right. Pass in an int from 0-100."""
        log.info('right(val=%d)' % val)
        self.right_x = val / 100.0

    def left(self, val):
        """Left tells the drone to go left. Pass in an int from 0-100."""
        log.info('left(val=%d)' % val)
        self.right_x = val / 100.0 * -1

    def clockwise(self, val):
        """
        Clockwise tells the drone to rotate in a clockwise direction.
        Pass in an int from 0-100.
        """
        log.info('clockwise(val=%d)' % val)
        self.left_x = val / 100.0

    def counter_clockwise(self, val):
        """
        CounterClockwise tells the drone to rotate in a counter-clockwise direction.
        Pass in an int from 0-100.
        """
        log.info('counter_clockwise(val=%d)' % val)
        self.left_x = val / 100.0 * -1

    def flip_forward(self):
        """flip_forward tells the drone to perform a forwards flip"""
        log.info('flip_forward (cmd=0x%02x seq=0x%04x)' % (FLIP_CMD, self.pkt_seq_num))
        pkt = Packet(FLIP_CMD, 0x70)
        pkt.add_byte(FlipFront)
        pkt.fixup()
        return self.send_packet(pkt)

    def flip_back(self):
        """flip_back tells the drone to perform a backwards flip"""
        log.info('flip_back (cmd=0x%02x seq=0x%04x)' % (FLIP_CMD, self.pkt_seq_num))
        pkt = Packet(FLIP_CMD, 0x70)
        pkt.add_byte(FlipBack)
        pkt.fixup()
        return self.send_packet(pkt)

    def flip_right(self):
        """flip_right tells the drone to perform a right flip"""
        log.info('flip_right (cmd=0x%02x seq=0x%04x)' % (FLIP_CMD, self.pkt_seq_num))
        pkt = Packet(FLIP_CMD, 0x70)
        pkt.add_byte(FlipRight)
        pkt.fixup()
        return self.send_packet(pkt)

    def flip_left(self):
        """flip_left tells the drone to perform a left flip"""
        log.info('flip_left (cmd=0x%02x seq=0x%04x)' % (FLIP_CMD, self.pkt_seq_num))
        pkt = Packet(FLIP_CMD, 0x70)
        pkt.add_byte(FlipLeft)
        pkt.fixup()
        return self.send_packet(pkt)

    def flip_forwardleft(self):
        """flip_forwardleft tells the drone to perform a forwards left flip"""
        log.info('flip_forwardleft (cmd=0x%02x seq=0x%04x)' % (FLIP_CMD, self.pkt_seq_num))
        pkt = Packet(FLIP_CMD, 0x70)
        pkt.add_byte(FlipForwardLeft)
        pkt.fixup()
        return self.send_packet(pkt)

    def flip_backleft(self):
        """flip_backleft tells the drone to perform a backwards left flip"""
        log.info('flip_backleft (cmd=0x%02x seq=0x%04x)' % (FLIP_CMD, self.pkt_seq_num))
        pkt = Packet(FLIP_CMD, 0x70)
        pkt.add_byte(FlipBackLeft)
        pkt.fixup()
        return self.send_packet(pkt)

    def flip_forwardright(self):
        """flip_forwardright tells the drone to perform a forwards right flip"""
        log.info('flip_forwardright (cmd=0x%02x seq=0x%04x)' % (FLIP_CMD, self.pkt_seq_num))
        pkt = Packet(FLIP_CMD, 0x70)
        pkt.add_byte(FlipForwardRight)
        pkt.fixup()
        return self.send_packet(pkt)

    def flip_backright(self):
        """flip_backleft tells the drone to perform a backwards right flip"""
        log.info('flip_backright (cmd=0x%02x seq=0x%04x)' % (FLIP_CMD, self.pkt_seq_num))
        pkt = Packet(FLIP_CMD, 0x70)
        pkt.add_byte(FlipBackRight)
        pkt.fixup()
        return self.send_packet(pkt)

    def __fix_range(self, val, min=-1.0, max=1.0):
        if val < min:
            val = min
        elif val > max:
            val = max
        return val

    def set_throttle(self, throttle):
        """
        Set_throttle controls the vertical up and down motion of the drone.
        Pass in an int from -1.0 ~ 1.0. (positive value means upward)
        """
        if self.left_y != self.__fix_range(throttle):
            log.info('set_throttle(val=%4.2f)' % throttle)
        self.left_y = self.__fix_range(throttle)

    def set_yaw(self, yaw):
        """
        Set_yaw controls the left and right rotation of the drone.
        Pass in an int from -1.0 ~ 1.0. (positive value will make the drone turn to the right)
        """
        if self.left_x != self.__fix_range(yaw):
            log.info('set_yaw(val=%4.2f)' % yaw)
        self.left_x = self.__fix_range(yaw)

    def set_pitch(self, pitch):
        """
        Set_pitch controls the forward and backward tilt of the drone.
        Pass in an int from -1.0 ~ 1.0. (positive value will make the drone move forward)
        """
        if self.right_y != self.__fix_range(pitch):
            log.info('set_pitch(val=%4.2f)' % pitch)
        self.right_y = self.__fix_range(pitch)

    def set_roll(self, roll):
        """
        Set_roll controls the the side to side tilt of the drone.
        Pass in an int from -1.0 ~ 1.0. (positive value will make the drone move to the right)
        """
        if self.right_x != self.__fix_range(roll):
            log.info('set_roll(val=%4.2f)' % roll)
        self.right_x = self.__fix_range(roll)

    def __send_stick_command(self):
        pkt = Packet(STICK_CMD, 0x60)

        axis1 = int(1024 + 660.0 * self.right_x) & 0x7ff
        axis2 = int(1024 + 660.0 * self.right_y) & 0x7ff
        axis3 = int(1024 + 660.0 * self.left_y) & 0x7ff
        axis4 = int(1024 + 660.0 * self.left_x) & 0x7ff
        '''
        11 bits (-1024 ~ +1023) x 4 axis = 44 bits
        44 bits will be packed in to 6 bytes (48 bits)

                    axis4      axis3      axis2      axis1
             |          |          |          |          |
                 4         3         2         1         0
        98765432109876543210987654321098765432109876543210
         |       |       |       |       |       |       |
             byte5   byte4   byte3   byte2   byte1   byte0
        '''
        log.debug("stick command: yaw=%4d thr=%4d pit=%4d rol=%4d" %
                  (axis4, axis3, axis2, axis1))
        log.debug("stick command: yaw=%04x thr=%04x pit=%04x rol=%04x" %
                  (axis4, axis3, axis2, axis1))
        pkt.add_byte(((axis2 << 11 | axis1) >> 0) & 0xff)
        pkt.add_byte(((axis2 << 11 | axis1) >> 8) & 0xff)
        pkt.add_byte(((axis3 << 11 | axis2) >> 5) & 0xff)
        pkt.add_byte(((axis4 << 11 | axis3) >> 2) & 0xff)
        pkt.add_byte(((axis4 << 11 | axis3) >> 10) & 0xff)
        pkt.add_byte(((axis4 << 11 | axis3) >> 18) & 0xff)
        pkt.add_time()
        pkt.fixup()
        log.debug("stick command: %s" % byte_to_hexstring(pkt.get_buffer()))
        return self.send_packet(pkt)

    def __send_ack_log(self, id):
        pkt = Packet(LOG_HEADER_MSG, 0x50)
        pkt.add_byte(0x00)
        b0, b1 = le16(id)
        pkt.add_byte(b0)
        pkt.add_byte(b1)
        pkt.fixup()
        return self.send_packet(pkt)

    def send_packet(self, pkt):
        """Send_packet is used to send a command packet to the drone."""
        try:
            cmd = pkt.get_buffer()
            self.sock.sendto(cmd, self.tello_addr)
            log.debug("send_packet: %s" % byte_to_hexstring(cmd))
        except socket.error as err:
            if self.state == self.STATE_CONNECTED:
                log.error("send_packet: %s" % str(err))
            else:
                log.info("send_packet: %s" % str(err))
            return False

        return True

    def send_packet_data(self, command, type=0x68, payload=[]):
        pkt = Packet(command, type, payload)
        pkt.fixup()
        return self.send_packet(pkt)

    def __process_packet(self, data):
        if isinstance(data, str):
            data = bytearray([x for x in data])

        if str(data[0:9]) == 'conn_ack:' or data[0:9] == b'conn_ack:':
            log.info('connected. (port=%2x%2x)' % (data[9], data[10]))
            log.debug('    %s' % byte_to_hexstring(data))
            if self.video_enabled:
                self.__send_exposure()
                self.__send_video_encoder_rate()
                self.__send_start_video()
            self.__publish(self.__EVENT_CONN_ACK, data)

            return True

        if data[0] != START_OF_PACKET:
            log.info('start of packet != %02x (%02x) (ignored)' % (START_OF_PACKET, data[0]))
            log.info('    %s' % byte_to_hexstring(data))
            log.info('    %s' % str(map(chr, data))[1:-1])
            return False

        pkt = Packet(data)
        cmd = uint16(data[5], data[6])
        if cmd == LOG_HEADER_MSG:
            id = uint16(data[9], data[10])
            log.info("recv: log_header: id=%04x, '%s'" % (id, str(data[28:54])))
            log.debug("recv: log_header: %s" % byte_to_hexstring(data[9:]))
            self.__send_ack_log(id)
            self.__publish(event=self.EVENT_LOG_HEADER, data=data[9:])
            if self.log_data_file and not self.log_data_header_recorded:
                self.log_data_file.write(data[12:-2])
                self.log_data_header_recorded = True
        elif cmd == LOG_DATA_MSG:
            log.debug("recv: log_data: length=%d, %s" % (len(data[9:]), byte_to_hexstring(data[9:])))
            self.__publish(event=self.EVENT_LOG_RAWDATA, data=data[9:])
            try:
                self.log_data.update(data[10:])
                if self.log_data_file:
                    self.log_data_file.write(data[10:-2])
            except Exception as ex:
                log.error('%s' % str(ex))
            self.__publish(event=self.EVENT_LOG_DATA, data=self.log_data)

        elif cmd == LOG_CONFIG_MSG:
            log.debug("recv: log_config: length=%d, %s" % (len(data[9:]), byte_to_hexstring(data[9:])))
            self.__publish(event=self.EVENT_LOG_CONFIG, data=data[9:])
        elif cmd == WIFI_MSG:
            log.debug("recv: wifi: %s" % byte_to_hexstring(data[9:]))
            self.wifi_strength = data[9]
            self.__publish(event=self.EVENT_WIFI, data=data[9:])
        elif cmd == LIGHT_MSG:
            log.debug("recv: light: %s" % byte_to_hexstring(data[9:]))
            self.__publish(event=self.EVENT_LIGHT, data=data[9:])
        elif cmd == FLIGHT_MSG:
            flight_data = FlightData(data[9:])
            flight_data.wifi_strength = self.wifi_strength
            log.debug("recv: flight data: %s" % str(flight_data))
            self.__publish(event=self.EVENT_FLIGHT_DATA, data=flight_data)
        elif cmd == TIME_CMD:
            log.debug("recv: time data: %s" % byte_to_hexstring(data))
            self.__publish(event=self.EVENT_TIME, data=data[7:9])
        elif cmd in (TAKEOFF_CMD, LAND_CMD, VIDEO_START_CMD, VIDEO_ENCODER_RATE_CMD, PALM_LAND_CMD,
                     EXPOSURE_CMD, THROW_AND_GO_CMD):
            log.info("recv: ack: cmd=0x%02x seq=0x%04x %s" %
                     (uint16(data[5], data[6]), uint16(data[7], data[8]), byte_to_hexstring(data)))
        elif cmd == TELLO_CMD_FILE_SIZE:
            # Drone is about to send us a file. Get ready.
            # N.b. one of the fields in the packet is a file ID; by demuxing
            # based on file ID we can receive multiple files at once. This
            # code doesn't support that yet, though, so don't take one photo
            # while another is still being received.
            log.info("recv: file size: %s" % byte_to_hexstring(data))
            if len(pkt.get_data()) >= 7:
                (size, filenum) = struct.unpack('<xLH', pkt.get_data())
                log.info('      file size: num=%d bytes=%d' % (filenum, size))
                # Initialize file download state.
                self.file_recv[filenum] = DownloadedFile(filenum, size)
            else:
                # We always seem to get two files, one with most of the payload missing.
                # Not sure what the second one is for.
                log.warn('      file size: payload too small: %s' % byte_to_hexstring(pkt.get_data()))
            # Ack the packet.
            self.send_packet(pkt)
        elif cmd == TELLO_CMD_FILE_DATA:
            # log.info("recv: file data: %s" % byte_to_hexstring(data[9:21]))
            # Drone is sending us a fragment of a file it told us to prepare
            # for earlier.
            self.recv_file_data(pkt.get_data())
        else:
            log.info('unknown packet: %04x %s' % (cmd, byte_to_hexstring(data)))
            return False

        return True

    def recv_file_data(self, data):
        (filenum,chunk,fragment,size) = struct.unpack('<HLLH', data[0:12])
        file = self.file_recv.get(filenum, None)

        # Preconditions.
        if file is None:
            return

        if file.recvFragment(chunk, fragment, size, data[12:12+size]):
            # Did this complete a chunk? Ack the chunk so the drone won't
            # re-send it.
            self.send_packet_data(TELLO_CMD_FILE_DATA, type=0x50,
                payload=struct.pack('<BHL', 0, filenum, chunk))

        if file.done():
            # We have the whole file! First, send a normal ack with the first
            # byte set to 1 to indicate file completion.
            self.send_packet_data(TELLO_CMD_FILE_DATA, type=0x50,
                payload=struct.pack('<BHL', 1, filenum, chunk))
            # Then send the FILE_COMPLETE packed separately telling it how
            # large we thought the file was.
            self.send_packet_data(TELLO_CMD_FILE_COMPLETE, type=0x48,
                payload=struct.pack('<HL', filenum, file.size))
            # Inform subscribers that we have a file and clean up.
            self.__publish(event=self.EVENT_FILE_RECEIVED, data=file.data())
            del self.file_recv[filenum]

    def record_log_data(self, path = None):
        if path == None:
            path = '%s/Documents/tello-%s.dat' % (
                os.getenv('HOME'),
                datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S'))
        log.info('record log data in %s' % path)
        self.log_data_file = open(path, 'wb')

    def __state_machine(self, event, sender, data, **args):
        self.lock.acquire()
        cur_state = self.state
        event_connected = False
        event_disconnected = False
        log.debug('event %s in state %s' % (str(event), str(self.state)))

        if self.state == self.STATE_DISCONNECTED:
            if event == self.__EVENT_CONN_REQ:
                self.__send_conn_req()
                self.state = self.STATE_CONNECTING
            elif event == self.__EVENT_QUIT_REQ:
                self.state = self.STATE_QUIT
                event_disconnected = True
                self.video_enabled = False

        elif self.state == self.STATE_CONNECTING:
            if event == self.__EVENT_CONN_ACK:
                self.state = self.STATE_CONNECTED
                event_connected = True
                # send time
                self.__send_time_command()
            elif event == self.__EVENT_TIMEOUT:
                self.__send_conn_req()
            elif event == self.__EVENT_QUIT_REQ:
                self.state = self.STATE_QUIT

        elif self.state == self.STATE_CONNECTED:
            if event == self.__EVENT_TIMEOUT:
                self.__send_conn_req()
                self.state = self.STATE_CONNECTING
                event_disconnected = True
                self.video_enabled = False
            elif event == self.__EVENT_QUIT_REQ:
                self.state = self.STATE_QUIT
                event_disconnected = True
                self.video_enabled = False

        elif self.state == self.STATE_QUIT:
            pass

        if cur_state != self.state:
            log.info('state transit %s -> %s' % (cur_state, self.state))
        self.lock.release()

        if event_connected:
            self.__publish(event=self.EVENT_CONNECTED, **args)
            self.connected.set()
        if event_disconnected:
            self.__publish(event=self.EVENT_DISCONNECTED, **args)
            self.connected.clear()

    def __recv_thread(self):
        sock = self.sock

        while self.state != self.STATE_QUIT:

            if self.state == self.STATE_CONNECTED:
                self.__send_stick_command()  # ignore errors

            try:
                data, server = sock.recvfrom(self.udpsize)
                log.debug("recv: %s" % byte_to_hexstring(data))
                self.__process_packet(data)
            except socket.timeout as ex:
                if self.state == self.STATE_CONNECTED:
                    log.error('recv: timeout')
                self.__publish(event=self.__EVENT_TIMEOUT)
            except Exception as ex:
                log.error('recv: %s' % str(ex))
                show_exception(ex)

        log.info('exit from the recv thread.')

    def __video_thread(self):
        log.info('start video thread')
        # Create a UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        port = 6038
        sock.bind(('', port))
        sock.settimeout(1.0)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 512 * 1024)
        log.info('video receive buffer size = %d' %
                 sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF))

        prev_video_data = None
        prev_ts = None
        history = []
        while self.state != self.STATE_QUIT:
            if not self.video_enabled:
                time.sleep(1.0)
                continue
            try:
                data, server = sock.recvfrom(self.udpsize)
                now = datetime.datetime.now()
                log.debug("video recv: %s %d bytes" % (byte_to_hexstring(data[0:2]), len(data)))
                show_history = False

                # check video data loss
                video_data = VideoData(data)
                loss = video_data.gap(prev_video_data)
                if loss != 0:
                    self.video_data_loss += loss
                    # enable this line to see packet history
                    # show_history = True
                prev_video_data = video_data

                # check video data interval
                if prev_ts is not None and 0.1 < (now - prev_ts).total_seconds():
                    log.info('video recv: %d bytes %02x%02x +%03d' %
                             (len(data), byte(data[0]), byte(data[1]),
                              (now - prev_ts).total_seconds() * 1000))
                prev_ts = now

                # save video data history
                history.append([now, len(data), byte(data[0])*256 + byte(data[1])])
                if 100 < len(history):
                    history = history[1:]

                # show video data history
                if show_history:
                    prev_ts = history[0][0]
                    for i in range(1, len(history)):
                        [ ts, sz, sn ] = history[i]
                        print('    %02d:%02d:%02d.%03d %4d bytes %04x +%03d%s' %
                              (ts.hour, ts.minute, ts.second, ts.microsecond/1000,
                               sz, sn, (ts - prev_ts).total_seconds()*1000,
                               (' *' if i == len(history) - 1 else '')))
                        prev_ts = ts
                    history = history[-1:]

                # deliver video frame to subscribers
                self.__publish(event=self.EVENT_VIDEO_FRAME, data=data[2:])
                self.__publish(event=self.EVENT_VIDEO_DATA, data=data)

                # show video frame statistics
                if self.prev_video_data_time is None:
                    self.prev_video_data_time = now
                self.video_data_size += len(data)
                dur = (now - self.prev_video_data_time).total_seconds()
                if 2.0 < dur:
                    log.info(('video data %d bytes %5.1fKB/sec' %
                              (self.video_data_size, self.video_data_size / dur / 1024)) +
                             ((' loss=%d' % self.video_data_loss) if self.video_data_loss != 0 else ''))
                    self.video_data_size = 0
                    self.prev_video_data_time = now
                    self.video_data_loss = 0

                    # keep sending start video command
                    self.__send_start_video()

            except socket.timeout as ex:
                log.error('video recv: timeout')
                self.start_video()
                data = None
            except Exception as ex:
                log.error('video recv: %s' % str(ex))
                show_exception(ex)

        log.info('exit from the video thread.')

if __name__ == '__main__':
    print('You can use test.py for testing.')
