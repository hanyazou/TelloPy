import datetime
import struct
import math
from io import BytesIO

from . import crc
from . utils import *

# low-level Protocol (https://tellopilots.com/wiki/protocol/#MessageIDs)
START_OF_PACKET                     = 0xcc
SSID_MSG                            = 0x0011
SSID_CMD                            = 0x0012
SSID_PASSWORD_MSG                   = 0x0013
SSID_PASSWORD_CMD                   = 0x0014
WIFI_REGION_MSG                     = 0x0015
WIFI_REGION_CMD                     = 0x0016
WIFI_MSG                            = 0x001a
VIDEO_ENCODER_RATE_CMD              = 0x0020
VIDEO_DYN_ADJ_RATE_CMD              = 0x0021
EIS_CMD                             = 0x0024
VIDEO_START_CMD                     = 0x0025
VIDEO_RATE_QUERY                    = 0x0028
TAKE_PICTURE_COMMAND                = 0x0030
VIDEO_MODE_CMD                      = 0x0031
VIDEO_RECORD_CMD                    = 0x0032
EXPOSURE_CMD                        = 0x0034
LIGHT_MSG                           = 0x0035
JPEG_QUALITY_MSG                    = 0x0037
ERROR_1_MSG                         = 0x0043
ERROR_2_MSG                         = 0x0044
VERSION_MSG                         = 0x0045
TIME_CMD                            = 0x0046
ACTIVATION_TIME_MSG                 = 0x0047
LOADER_VERSION_MSG                  = 0x0049
STICK_CMD                           = 0x0050
TAKEOFF_CMD                         = 0x0054
LAND_CMD                            = 0x0055
FLIGHT_MSG                          = 0x0056
SET_ALT_LIMIT_CMD                   = 0x0058
FLIP_CMD                            = 0x005c
THROW_AND_GO_CMD                    = 0x005d
PALM_LAND_CMD                       = 0x005e
TELLO_CMD_FILE_SIZE                 = 0x0062  # pt50
TELLO_CMD_FILE_DATA                 = 0x0063  # pt50
TELLO_CMD_FILE_COMPLETE             = 0x0064  # pt48
SMART_VIDEO_CMD                     = 0x0080
SMART_VIDEO_STATUS_MSG              = 0x0081
LOG_HEADER_MSG                      = 0x1050
LOG_DATA_MSG                        = 0x1051
LOG_CONFIG_MSG                      = 0x1052
BOUNCE_CMD                          = 0x1053
CALIBRATE_CMD                       = 0x1054
LOW_BAT_THRESHOLD_CMD               = 0x1055
ALT_LIMIT_MSG                       = 0x1056
LOW_BAT_THRESHOLD_MSG               = 0x1057
ATT_LIMIT_CMD                       = 0x1058 # Stated incorrectly by Wiki (checked from raw packets)
ATT_LIMIT_MSG                       = 0x1059

EMERGENCY_CMD                       = 'emergency'

#Flip commands taken from Go version of code
#FlipFront flips forward.
FlipFront = 0
#FlipLeft flips left.
FlipLeft = 1
#FlipBack flips backwards.
FlipBack = 2
#FlipRight flips to the right.
FlipRight = 3
#FlipForwardLeft flips forwards and to the left.
FlipForwardLeft = 4
#FlipBackLeft flips backwards and to the left.
FlipBackLeft = 5
#FlipBackRight flips backwards and to the right.
FlipBackRight = 6
#FlipForwardRight flips forwards and to the right.
FlipForwardRight = 7

class Packet(object):
    def __init__(self, cmd, pkt_type=0x68, payload=b''):
        if isinstance(cmd, str):
            self.buf = bytearray()
            for c in cmd:
                self.buf.append(ord(c))
        elif isinstance(cmd, (bytearray, bytes)):
            self.buf = bytearray()
            self.buf[:] = cmd
        else:
            self.buf = bytearray([
                START_OF_PACKET,
                0, 0,
                0,
                pkt_type,
                (cmd & 0xff), ((cmd >> 8) & 0xff),
                0, 0])
            self.buf.extend(payload)

    def fixup(self, seq_num=0):
        buf = self.get_buffer()
        if buf[0] == START_OF_PACKET:
            buf[1], buf[2] = le16(len(buf)+2)
            buf[1] = (buf[1] << 3)
            buf[3] = crc.crc8(buf[0:3])
            buf[7], buf[8] = le16(seq_num)
            self.add_int16(crc.crc16(buf))

    def get_buffer(self):
        return self.buf

    def get_data(self):
        return self.buf[9:len(self.buf)-2]

    def add_byte(self, val):
        self.buf.append(val & 0xff)

    def add_int16(self, val):
        self.add_byte(val)
        self.add_byte(val >> 8)

    def add_time(self, time=datetime.datetime.now()):
        self.add_int16(time.hour)
        self.add_int16(time.minute)
        self.add_int16(time.second)
        self.add_int16(int(time.microsecond/1000) & 0xff)
        self.add_int16((int(time.microsecond/1000) >> 8) & 0xff)

    def get_time(self, buf=None):
        if buf is None:
            buf = self.get_data()[1:]
        hour = int16(buf[0], buf[1])
        min = int16(buf[2], buf[3])
        sec = int16(buf[4], buf[5])
        millisec = int16(buf[6], buf[8])
        now = datetime.datetime.now()
        return datetime.datetime(now.year, now.month, now.day, hour, min, sec, millisec)


class FlightData(object):
    def __init__(self, data):
        self.battery_low = 0
        self.battery_lower = 0
        self.battery_percentage = 0
        self.battery_state = 0
        self.camera_state = 0
        self.down_visual_state = 0
        self.drone_battery_left = 0
        self.drone_fly_time_left = 0
        self.drone_hover = 0
        self.em_open = 0
        self.em_sky = 0
        self.em_ground = 0
        self.east_speed = 0
        self.electrical_machinery_state = 0
        self.factory_mode = 0
        self.fly_mode = 0
        self.fly_speed = 0
        self.fly_time = 0
        self.front_in = 0
        self.front_lsc = 0
        self.front_out = 0
        self.gravity_state = 0
        self.ground_speed = 0
        self.height = 0
        self.imu_calibration_state = 0
        self.imu_state = 0
        self.light_strength = 0
        self.north_speed = 0
        self.outage_recording = 0
        self.power_state = 0
        self.pressure_state = 0
        self.smart_video_exit_mode = 0
        self.temperature_height = 0
        self.throw_fly_timer = 0
        self.wifi_disturb = 0
        self.wifi_strength = 0
        self.wind_state = 0

        if len(data) < 24:
            return

        self.height = int16(data[0], data[1])
        self.north_speed = int16(data[2], data[3])
        self.east_speed = int16(data[4], data[5])
        self.ground_speed = int16(data[6], data[7])
        self.fly_time = int16(data[8], data[9])

        self.imu_state = ((data[10] >> 0) & 0x1)
        self.pressure_state = ((data[10] >> 1) & 0x1)
        self.down_visual_state = ((data[10] >> 2) & 0x1)
        self.power_state = ((data[10] >> 3) & 0x1)
        self.battery_state = ((data[10] >> 4) & 0x1)
        self.gravity_state = ((data[10] >> 5) & 0x1)
        self.wind_state = ((data[10] >> 7) & 0x1)

        self.imu_calibration_state = data[11]
        self.battery_percentage = data[12]
        self.drone_battery_left = int16(data[13], data[14])
        self.drone_fly_time_left = int16(data[15], data[16])

        self.em_sky = ((data[17] >> 0) & 0x1)
        self.em_ground = ((data[17] >> 1) & 0x1)
        self.em_open = ((data[17] >> 2) & 0x1)
        self.drone_hover = ((data[17] >> 3) & 0x1)
        self.outage_recording = ((data[17] >> 4) & 0x1)
        self.battery_low = ((data[17] >> 5) & 0x1)
        self.battery_lower = ((data[17] >> 6) & 0x1)
        self.factory_mode = ((data[17] >> 7) & 0x1)

        self.fly_mode = data[18]
        self.throw_fly_timer = data[19]
        self.camera_state = data[20]
        self.electrical_machinery_state = data[21]

        self.front_in = ((data[22] >> 0) & 0x1)
        self.front_out = ((data[22] >> 1) & 0x1)
        self.front_lsc = ((data[22] >> 2) & 0x1)

        self.temperature_height = ((data[23] >> 0) & 0x1)

    def __str__(self):
        return (
            ("ALT: %2d" % self.height) +
            (" | SPD: %2d" % self.ground_speed) +
            (" | BAT: %2d" % self.battery_percentage) +
            (" | WIFI: %2d" % self.wifi_strength) +
            (" | CAM: %2d" % self.camera_state) +
            (" | MODE: %2d" % self.fly_mode) +
            # (", drone_battery_left=0x%04x" % self.drone_battery_left) +
            "")

class DownloadedFile(object):
    def __init__(self, filenum, size):
        self.filenum = filenum
        self.size = size
        self.bytes_recieved = 0
        self.chunks_received = [0x00] * int((size / 1024 + 1) / 8 + 1)
        self.buffer = BytesIO()

    def done(self):
        return self.bytes_recieved >= self.size

    def data(self):
        return self.buffer.getvalue()

    def haveFragment(self, chunk, fragment):
        return self.chunks_received[chunk] & (1<<(fragment%8))

    def recvFragment(self, chunk, fragment, size, data):
        if self.haveFragment(chunk, fragment):
            return False
        # Mark a fragment as received.
        # Returns true if we have all fragments making up that chunk now.
        self.buffer.seek(fragment*1024)
        self.buffer.write(data)
        self.bytes_recieved += size
        self.chunks_received[chunk] |= (1<<(fragment%8))
        return self.chunks_received[chunk] == 0xFF


class VideoData(object):
    packets_per_frame = 0
    def __init__(self, data):
        self.h0 = byte(data[0])
        self.h1 = byte(data[1])
        if VideoData.packets_per_frame < (self.h1 & 0x7f):
            VideoData.packets_per_frame = (self.h1 & 0x7f)

    def gap(self, video_data):
        if video_data is None:
            return 0

        v0 = self
        v1 = video_data

        loss = 0
        if ((v0.h0 != v1.h0 and v0.h0 != ((v1.h0 + 1) & 0xff))
            or (v0.h0 != v1.h0 and (v0.h1 & 0x7f) != 00)
            or (v0.h0 == v1.h0 and (v0.h1 & 0x7f) != (v1.h1 & 0x7f) + 1)):
            loss = v0.h0 - v1.h0
            if loss < 0:
                loss = loss + 256
            loss = loss * VideoData.packets_per_frame + ((v0.h1 & 0x7f) - (v1.h1 & 0x7f) - 1)

        return loss


class LogData(object):
    ID_NEW_MVO_FEEDBACK                = 29
    ID_IMU_ATTI                        = 2048
    unknowns = []

    def __init__(self, log, data = None):
        self.log = log
        self.count = 0
        self.mvo = LogNewMvoFeedback(log)
        self.imu = LogImuAtti(log)
        if data:
            self.update(data)

    def __str__(self):
        return ('MVO: ' + str(self.mvo) +
                '|IMU: ' + str(self.imu) +
                "")

    def format_cvs(self):
        return (
            self.mvo.format_cvs() +
            ',' + self.imu.format_cvs() +
            "")

    def format_cvs_header(self):
        return (
            self.mvo.format_cvs_header() +
            ',' + self.imu.format_cvs_header() +
            "")

    def update(self, data):
        if isinstance(data, bytearray):
            data = str(data)

        self.log.debug('LogData: data length=%d' % len(data))
        self.count += 1
        pos = 0
        while (pos < len(data) - 2):
            if (struct.unpack_from('B', data, pos+0)[0] != 0x55):
                raise Exception('LogData: corrupted data at pos=%d, data=%s'
                               % (pos, byte_to_hexstring(data[pos:])))
            length = struct.unpack_from('<h', data, pos+1)[0]
            checksum = data[pos+3]
            id = struct.unpack_from('<H', data, pos+4)[0]
            # 4bytes data[6:9] is tick
            # last 2 bytes are CRC
            # length-12 is the byte length of payload
            xorval = data[pos+6]
            if isinstance(data, str):
                payload = bytearray([ord(x) ^ ord(xorval) for x in data[pos+10:pos+10+length-12]])
            else:
                payload = bytearray([x ^ xorval for x in data[pos+10:pos+10+length-12]])
            if id == self.ID_NEW_MVO_FEEDBACK:
                self.mvo.update(payload, self.count)
            elif id == self.ID_IMU_ATTI:
                self.imu.update(payload, self.count)
            else:
                if not id in self.unknowns:
                    self.log.info('LogData: UNHANDLED LOG DATA: id=%5d, length=%4d' % (id, length-12))
                    self.unknowns.append(id)

            pos += length
        if pos != len(data) - 2:
            raise Exception('LogData: corrupted data at pos=%d, data=%s'
                            % (pos, byte_to_hexstring(data[pos:])))


class LogNewMvoFeedback(object):
    def __init__(self, log = None, data = None):
        self.log = log
        self.count = 0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_z = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        if (data != None):
            self.update(data, count)

    def __str__(self):
        return (
            ("VEL: %5.2f %5.2f %5.2f" % (self.vel_x, self.vel_y, self.vel_z))+
            (" POS: %5.2f %5.2f %5.2f" % (self.pos_x, self.pos_y, self.pos_z))+
            "")

    def format_cvs(self):
        return (
            ("%f,%f,%f" % (self.vel_x, self.vel_y, self.vel_z))+
            (",%f,%f,%f" % (self.pos_x, self.pos_y, self.pos_z))+
            "")

    def format_cvs_header(self):
        return (
            "mvo.vel_x,mvo.vel_y,mvo.vel_z" +
            ",mvo.pos_x,mvo.pos_y,mvo.pos_z" +
            "")

    def update(self, data, count = 0):
        self.log.debug('LogNewMvoFeedback: length=%d %s' % (len(data), byte_to_hexstring(data)))
        self.count = count
        (self.vel_x, self.vel_y, self.vel_z) = struct.unpack_from('<hhh', data, 2)
        self.vel_x /= 100.0
        self.vel_y /= 100.0
        self.vel_z /= 100.0
        (self.pos_x, self.pos_y, self.pos_z) = struct.unpack_from('fff', data, 8)
        self.log.debug('LogNewMvoFeedback: ' + str(self))


class LogImuAtti(object):
    def __init__(self, log = None, data = None):
        self.log = log
        self.count = 0
        self.acc_x = 0.0
        self.acc_y = 0.0
        self.acc_z = 0.0
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        self.q0 = 0.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        self.yaw = 0.0
        self.vg_x = 0.0
        self.vg_y = 0.0
        self.vg_z = 0.0
        if (data != None):
            self.update(data)

    def __str__(self):
        return (
            ("ACC: %5.2f %5.2f %5.2f" % (self.acc_x, self.acc_y, self.acc_z)) +
            (" GYRO: %5.2f %5.2f %5.2f" % (self.gyro_x, self.gyro_y, self.gyro_z)) +
            (" QUATERNION: %5.2f %5.2f %5.2f %5.2f" % (self.q0, self.q1, self.q2, self.q3)) +
            (" VG: %5.2f %5.2f %5.2f" % (self.vg_x, self.vg_y, self.vg_z)) +
            "")

    def format_cvs(self):
        return (
            ("%f,%f,%f" % (self.acc_x, self.acc_y, self.acc_z)) +
            (",%f,%f,%f" % (self.gyro_x, self.gyro_y, self.gyro_z)) +
            (",%f,%f,%f,%f" % (self.q0, self.q1, self.q2, self.q3)) +
            (",%f,%f,%f" % (self.vg_x, self.vg_y, self.vg_z)) +
            "")

    def format_cvs_header(self):
        return (
            "imu.acc_x,imu.acc_y,imu.acc_z" +
            ",imu.gyro_x,imu.gyro_y,imu.gyro_z" +
            ",imu.q0,imu.q1,imu.q2, self.q3" +
            ",imu.vg_x,imu.vg_y,imu.vg_z" +
            "")

    def quaternion_to_yaw_degree(self, qw, qx, qy, qz):
        degree = math.pi / 180.0
        sin_y = 2.0 * (qw * qz + qx * qy)
        cos_y = 1.0 - 2 * (qy * qy + qz * qz)
        yaw = round(math.atan2(sin_y, cos_y) / degree, 2)
        return yaw

    def update(self, data, count = 0):
        self.log.debug('LogImuAtti: length=%d %s' % (len(data), byte_to_hexstring(data)))
        self.count = count
        (self.acc_x, self.acc_y, self.acc_z) = struct.unpack_from('fff', data, 20)
        (self.gyro_x, self.gyro_y, self.gyro_z) = struct.unpack_from('fff', data, 32)
        (self.q0, self.q1, self.q2, self.q3) = struct.unpack_from('ffff', data, 48)
        (self.vg_x, self.vg_y, self.vg_z) = struct.unpack_from('fff', data, 76)
        self.yaw = self.quaternion_to_yaw_degree(self.q0, self.q1, self.q2, self.q3)
        self.log.debug('LogImuAtti: ' + str(self))
