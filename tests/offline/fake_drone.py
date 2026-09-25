"""A stand-in for the drone, on loopback UDP.

It plays the drone's side of just enough of the protocol to let a real
Tello connect, get its commands acknowledged, and receive telemetry and
video. Packets it builds are put together here from the wire format, not
with the library's Packet.fixup() (which can't build long packets, and
shouldn't be what checks itself). The CRC tables are still the library's,
though, so a mistake in those would go unnoticed here.
"""
import collections
import socket
import struct
import threading
import time

from tellopy._internal import crc
from tellopy._internal.protocol import (
    START_OF_PACKET, STICK_CMD, LOG_HEADER_MSG, LOG_DATA_MSG, LOG_CONFIG_MSG,
    WIFI_MSG, LIGHT_MSG, FLIGHT_MSG, TIME_CMD, CALIBRATION_STATUS_CMD)

# What we got from the Tello. crc_ok says whether both CRCs in the header
# and trailer were right; payload is what sits between them.
Received = collections.namedtuple('Received', 'cmd seq pkt_type payload crc_ok time')

# Commands the real drone doesn't answer with an echo (the continuous stick
# command, and our acknowledgement of a log header).
NO_ACK = (STICK_CMD, LOG_HEADER_MSG)


def build_packet(cmd, payload=b'', seq=0, pkt_type=0x50):
    total = 9 + len(payload) + 2
    buf = bytearray([START_OF_PACKET]) + bytearray(struct.pack('<H', total << 3))
    buf.append(crc.crc8(buf[0:3]))
    buf.append(pkt_type)
    buf += struct.pack('<HH', cmd, seq)
    buf += payload
    buf += struct.pack('<H', crc.crc16(buf))
    return bytes(buf)


def log_record(record_id, tick, payload):
    """One record of a log data message, framed as LogData expects."""
    header = struct.pack('<BhBHI', 0x55, 12 + len(payload), 0, record_id, tick)
    key = header[6]     # the low byte of tick is the XOR key
    return header + bytes(b ^ key for b in payload) + b'\x00\x00'


class FakeDrone(object):
    def __init__(self, video_port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 0))
        self.sock.settimeout(0.05)
        self.address = self.sock.getsockname()
        self.video_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.video_address = ('127.0.0.1', video_port)
        self.client = None                  # where the Tello is, learned from its first packet
        self.received = []
        self.ack_enabled = True             # echo each command back, as the real drone does
        self.lock = threading.Lock()
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self.__run, name='FakeDrone')
        self.thread.daemon = True
        self.thread.start()

    def stop(self):
        self.stopped.set()
        self.thread.join(5.0)
        self.sock.close()
        self.video_sock.close()

    def received_cmds(self, cmd):
        with self.lock:
            return [r for r in self.received if r.cmd == cmd]

    def wait_for_cmd(self, cmd, count=1, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            found = self.received_cmds(cmd)
            if len(found) >= count:
                return found
            time.sleep(0.005)
        raise AssertionError('fake drone did not get %d x cmd 0x%04x within %.1fs (got %d)' %
                             (count, cmd, timeout, len(self.received_cmds(cmd))))

    # -- sending to the Tello --------------------------------------------

    def send(self, cmd, payload=b'', seq=0):
        self.sock.sendto(build_packet(cmd, payload, seq), self.client)

    def send_wifi(self):
        self.send(WIFI_MSG, bytes([80, 0]))

    def send_light(self):
        self.send(LIGHT_MSG, bytes([1, 0]))

    def send_flight_data(self, battery=0):
        payload = bytearray(24)
        payload[12] = battery           # battery_percentage
        self.send(FLIGHT_MSG, bytes(payload))

    def send_time(self):
        self.send(TIME_CMD, bytes(2))

    def send_log_header(self, log_id=0x1234):
        self.send(LOG_HEADER_MSG, struct.pack('<H', log_id) + bytes(60))

    def send_log_config(self):
        self.send(LOG_CONFIG_MSG, bytes(8))

    def send_log_data(self, *records):
        self.send(LOG_DATA_MSG, b'\x00' + b''.join(records))

    def send_calibration_status(self, step_mask=0, progress=0):
        self.send(CALIBRATION_STATUS_CMD, bytes([0, step_mask, 0, progress, 0]))

    def send_video(self, h0, h1, body=b''):
        self.video_sock.sendto(bytes([h0, h1]) + body, self.video_address)

    def poke(self, tello_port):
        """Send junk to wake up a Tello thread that is blocked on receive,
        so it notices a quit right away instead of after its timeout."""
        self.sock.sendto(b'x', ('127.0.0.1', tello_port))
        self.video_sock.sendto(b'x' * 4, self.video_address)

    # -- the drone's side of the conversation ------------------------------

    def __run(self):
        while not self.stopped.is_set():
            try:
                data, source = self.sock.recvfrom(2000)
            except socket.timeout:
                continue
            except OSError:
                return
            self.__handle(data, source)

    def __handle(self, data, source):
        if data.startswith(b'conn_req:'):
            self.client = source
            self.sock.sendto(b'conn_ack:' + bytes([0x96, 0x17]), source)
            return
        if data[0] != START_OF_PACKET:
            return
        buf = bytearray(data)
        self.client = source
        cmd, seq = struct.unpack_from('<HH', buf, 5)
        crc_ok = (buf[3] == crc.crc8(buf[0:3]) and
                  struct.unpack_from('<H', buf, len(buf) - 2)[0] == crc.crc16(buf[:-2]))
        with self.lock:
            self.received.append(Received(cmd, seq, buf[4], bytes(buf[9:-2]), crc_ok, time.monotonic()))
        if self.ack_enabled and cmd not in NO_ACK:
            self.send(cmd, bytes(buf[9:-2]), seq)
