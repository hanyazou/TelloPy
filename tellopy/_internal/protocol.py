import datetime
import struct
from io import BytesIO

from . import crc
from . utils import *
from . sample import Sample

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
CALIBRATION_START_CMD               = 0x005a
CALIBRATION_STATUS_CMD              = 0x005b
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
# CALIBRATE_CMD                     = 0x1054 # Use CALIBRATION_START_CMD instead.
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


class CalibrationStatus(object):
    """Decodes the 5-byte payload of a CALIBRATION_STATUS_CMD reply.

    - `step_mask` (byte[1]): a bitmask that gains one more set bit each
      time the drone accepts a new, sufficiently distinct orientation. Goes
      0 -> ... -> 0x3f (all 6 bits set) over a full calibration, regardless
      of the order orientations are presented in.
    - `progress` (byte[3]): a coarser 0-100 "calibration data sufficiency"
      score. Reaches 100 at/after `step_mask` reaches 0x3f.
    """
    def __init__(self, data):
        self.raw = bytes(data)
        self.step_mask = data[1] if len(data) > 1 else 0
        self.progress = data[3] if len(data) > 3 else 0

    @property
    def steps_done(self):
        """Number of set bits in step_mask (popcount)."""
        return bin(self.step_mask & 0xff).count('1')

    @property
    def done(self):
        return self.progress >= 100

    def __str__(self):
        return (
            "progress=%3d%% steps=%d/6 (mask=0b%s) raw=%s"
            % (
                self.progress,
                self.steps_done,
                bin(self.step_mask)[2:].zfill(6),
                byte_to_hexstring(self.raw),
            )
        )


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


class LogRecord(Sample):
    """One record of a log data message (cmd 0x1051).

    tick is the device counter the record carries, and recv_time is when
    the message that held it arrived. event_time is provisional, set to
    recv_time: the best the library alone can say, and only an upper
    bound on when the measurement was actually taken. A better estimate
    needs a tick-to-host-time model, which lives outside the library.

    Records whose id has no dedicated subclass are represented by this
    class as-is (record_id says which). payload is kept on every record,
    decoded or not, so fields identified later can be decoded from a
    capture without having to record it again.

    LogData makes a fresh record for every message it parses, rather than
    updating one in place, so a record that has been published never
    changes afterwards.
    """
    ID = None

    def __init__(self, log = None, data = None):
        super(LogRecord, self).__init__(event_time=None, tick=0, recv_time=None)
        self.log = log
        self.count = 0
        self.record_id = self.ID
        self.payload = b''
        if (data != None):
            self.update(data)

    def __str__(self):
        return "RECORD: %s TICK: %d" % (self.record_id, self.tick)

    def update(self, data, count = 0, tick = 0, recv_time = None):
        self.count = count
        self.tick = tick
        self.recv_time = recv_time
        self.event_time = recv_time
        self.payload = bytes(data)
        self.decode(data)

    def decode(self, data):
        """Fill in this record's fields from its payload; nothing to
        decode for a record we don't know yet."""
        pass


class LogNewMvoFeedback(LogRecord):
    ID = 29

    def __init__(self, log = None, data = None):
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_z = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        super(LogNewMvoFeedback, self).__init__(log, data)

    def __str__(self):
        return (
            ("TICK: %d" % self.tick) +
            (" VEL: %5.2f %5.2f %5.2f" % (self.vel_x, self.vel_y, self.vel_z))+
            (" POS: %5.2f %5.2f %5.2f" % (self.pos_x, self.pos_y, self.pos_z))+
            "")

    def format_cvs(self):
        return (
            ("%d" % self.tick) +
            (",%f,%f,%f" % (self.vel_x, self.vel_y, self.vel_z))+
            (",%f,%f,%f" % (self.pos_x, self.pos_y, self.pos_z))+
            "")

    def format_cvs_header(self):
        return (
            "mvo.tick" +
            ",mvo.vel_x,mvo.vel_y,mvo.vel_z" +
            ",mvo.pos_x,mvo.pos_y,mvo.pos_z" +
            "")

    def decode(self, data):
        self.log.debug('LogNewMvoFeedback: length=%d %s' % (len(data), byte_to_hexstring(data)))
        (self.vel_x, self.vel_y, self.vel_z) = struct.unpack_from('<hhh', data, 2)
        self.vel_x /= 100.0
        self.vel_y /= 100.0
        self.vel_z /= 100.0
        (self.pos_x, self.pos_y, self.pos_z) = struct.unpack_from('fff', data, 8)
        self.log.debug('LogNewMvoFeedback: ' + str(self))


class LogImuAtti(LogRecord):
    ID = 2048

    def __init__(self, log = None, data = None):
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
        # World-frame, gravity-compensated linear acceleration in m/s^2
        # (correlates 0.95-0.98 with the quaternion-rotated acc).
        self.lin_acc_x = 0.0
        self.lin_acc_y = 0.0
        self.lin_acc_z = 0.0
        # Not identified; the name is just what it has always been called.
        self.vg_x = 0.0
        self.vg_y = 0.0
        self.vg_z = 0.0
        super(LogImuAtti, self).__init__(log, data)

    def __str__(self):
        return (
            ("TICK: %d" % self.tick) +
            (" ACC: %5.2f %5.2f %5.2f" % (self.acc_x, self.acc_y, self.acc_z)) +
            (" GYRO: %5.2f %5.2f %5.2f" % (self.gyro_x, self.gyro_y, self.gyro_z)) +
            (" QUATERNION: %5.2f %5.2f %5.2f %5.2f" % (self.q0, self.q1, self.q2, self.q3)) +
            (" VG: %5.2f %5.2f %5.2f" % (self.vg_x, self.vg_y, self.vg_z)) +
            "")

    def format_cvs(self):
        return (
            ("%d" % self.tick) +
            (",%f,%f,%f" % (self.acc_x, self.acc_y, self.acc_z)) +
            (",%f,%f,%f" % (self.gyro_x, self.gyro_y, self.gyro_z)) +
            (",%f,%f,%f,%f" % (self.q0, self.q1, self.q2, self.q3)) +
            (",%f,%f,%f" % (self.vg_x, self.vg_y, self.vg_z)) +
            "")

    def format_cvs_header(self):
        return (
            "imu.tick" +
            ",imu.acc_x,imu.acc_y,imu.acc_z" +
            ",imu.gyro_x,imu.gyro_y,imu.gyro_z" +
            ",imu.q0,imu.q1,imu.q2, self.q3" +
            ",imu.vg_x,imu.vg_y,imu.vg_z" +
            "")

    def decode(self, data):
        self.log.debug('LogImuAtti: length=%d %s' % (len(data), byte_to_hexstring(data)))
        (self.acc_x, self.acc_y, self.acc_z) = struct.unpack_from('fff', data, 20)
        (self.gyro_x, self.gyro_y, self.gyro_z) = struct.unpack_from('fff', data, 32)
        (self.q0, self.q1, self.q2, self.q3) = struct.unpack_from('ffff', data, 48)
        (self.lin_acc_x, self.lin_acc_y, self.lin_acc_z) = struct.unpack_from('fff', data, 64)
        (self.vg_x, self.vg_y, self.vg_z) = struct.unpack_from('fff', data, 76)
        self.log.debug('LogImuAtti: ' + str(self))


class LogGyro(LogRecord):
    """Three pipeline stages of 3-axis gyroscope readings (~20Hz).

    Each stage tracks LogImuAtti's gyro closely (z: corr 0.96-0.99) but
    they lead/lag one another by up to a few tens of ms. Which stage is
    rawest is not known, hence the neutral stages[0..2]. x/y are only
    weakly correlated with LogImuAtti's gyro (small signals, mostly
    noise), so the x/y/z order within a stage is inferred from the z
    offsets.
    """
    ID = 1305

    def __init__(self, log = None, data = None):
        self.stages = ((0.0, 0.0, 0.0),) * 3
        super(LogGyro, self).__init__(log, data)

    def decode(self, data):
        self.stages = tuple(struct.unpack_from('<3f', data, off) for off in (1, 13, 25))


class LogTof(LogRecord):
    """Raw, uncompensated time-of-flight distance sensor (~5Hz)."""
    ID = 16

    def __init__(self, log = None, data = None):
        self.distance = 0       # unit not confirmed
        self.flag = 0           # only ever seen as 1
        self.counter = 0        # advances by 4 per record, wraps at 256
        super(LogTof, self).__init__(log, data)

    def decode(self, data):
        (self.distance, self.flag, self.counter) = struct.unpack_from('<hBB', data, 0)


class LogControl(LogRecord):
    """Internal flight controller quantities (~20Hz).

    cols holds all eight int16 columns. Per docs/sensor_time.md, column 0
    and 2 are the roll and yaw control errors, column 3 is throttle
    output, and column 1 (a pitch candidate) is unconfirmed. Column 4 is
    a copy of column 0 (identical in 99.9% of the records of the one
    capture checked); 5-7 are unidentified.
    """
    ID = 1306

    def __init__(self, log = None, data = None):
        self.cols = (0,) * 8
        super(LogControl, self).__init__(log, data)

    def decode(self, data):
        self.cols = struct.unpack_from('<8h', data, 6)

    @property
    def roll_err(self):
        return self.cols[0]

    @property
    def yaw_err(self):
        return self.cols[2]

    @property
    def throttle(self):
        return self.cols[3]


class LogData(object):
    ID_NEW_MVO_FEEDBACK                = LogNewMvoFeedback.ID
    ID_IMU_ATTI                        = LogImuAtti.ID
    RECORD_CLASSES = dict((cls.ID, cls) for cls in (
        LogNewMvoFeedback, LogImuAtti, LogGyro, LogTof, LogControl))
    unknowns = []

    def __init__(self, log, data = None):
        self.log = log
        self.count = 0
        self.recv_time = 0.0
        # Every record the last update() parsed, in order (a fresh object
        # each; see LogRecord). If update() raised partway, this still
        # holds the records that were fine before the corrupt spot.
        self.records = []
        # The most recent record of each kind seen so far, across messages.
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
            ("%.6f" % self.recv_time) +
            ',' + self.mvo.format_cvs() +
            ',' + self.imu.format_cvs() +
            "")

    def format_cvs_header(self):
        return (
            "recv_time" +
            ',' + self.mvo.format_cvs_header() +
            ',' + self.imu.format_cvs_header() +
            "")

    def update(self, data, recv_time=None):
        if isinstance(data, bytearray):
            data = str(data)

        if recv_time is not None:
            self.recv_time = recv_time
        self.log.debug('LogData: data length=%d' % len(data))
        self.count += 1
        self.records = []
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
            tick = struct.unpack_from('<I', data, pos+6)[0]
            xorval = data[pos+6]
            if isinstance(data, str):
                payload = bytearray([ord(x) ^ ord(xorval) for x in data[pos+10:pos+10+length-12]])
            else:
                payload = bytearray([x ^ xorval for x in data[pos+10:pos+10+length-12]])
            cls = self.RECORD_CLASSES.get(id)
            if cls is None:
                if not id in self.unknowns:
                    self.log.info('LogData: UNHANDLED LOG DATA: id=%5d, length=%4d' % (id, length-12))
                    self.unknowns.append(id)
                record = LogRecord(self.log)
                record.record_id = id
            else:
                record = cls(self.log)
            try:
                record.update(payload, self.count, tick, self.recv_time)
            except struct.error as ex:
                # A record shorter than its decoder expects; drop just
                # this one so the rest of the message still gets through.
                self.log.info('LogData: bad record id=%d: %s' % (id, str(ex)))
            else:
                self.records.append(record)
                if id == self.ID_NEW_MVO_FEEDBACK:
                    self.mvo = record
                elif id == self.ID_IMU_ATTI:
                    self.imu = record

            pos += length
        if pos != len(data) - 2:
            raise Exception('LogData: corrupted data at pos=%d, data=%s'
                            % (pos, byte_to_hexstring(data[pos:])))
