"""Connecting, commands and their acknowledgements, video, and quitting,
against a FakeDrone."""
import time

from tellopy._internal import protocol

from .harness import DroneTestCase, wait_until


class ConnectionTest(DroneTestCase):

    def test_connect_handshake_then_time_is_sent(self):
        self.connect()
        found = self.fake.wait_for_cmd(protocol.TIME_CMD)
        self.assertTrue(found[0].crc_ok)

    def test_quit_reports_disconnect_once(self):
        drone = self.start_drone()
        calls = []
        drone.subscribe(drone.EVENT_DISCONNECTED, lambda event, sender, data: calls.append(event))
        self.connect()
        drone.quit()
        drone.quit()
        self.assertEqual(len(calls), 1)


class CommandTest(DroneTestCase):

    def test_every_packet_sent_has_a_valid_crc_and_a_distinct_seq(self):
        drone = self.connect()
        for _ in range(20):
            drone.land()
        # let stick commands go out too: one is sent per packet received
        for _ in range(10):
            self.fake.send_flight_data()
            time.sleep(0.01)
        self.fake.wait_for_cmd(protocol.LAND_CMD, count=20)
        self.fake.wait_for_cmd(protocol.STICK_CMD, count=3)
        with self.fake.lock:
            received = list(self.fake.received)
        self.assertTrue(all(r.crc_ok for r in received))
        seqs = [r.seq for r in received]
        self.assertNotIn(0, seqs)
        self.assertEqual(len(seqs), len(set(seqs)), 'a seq number was used twice')


class VideoTest(DroneTestCase):

    def test_video_stream_delivers_the_bytes_sent(self):
        drone = self.connect()
        stream = drone.get_video_stream()
        self.fake.wait_for_cmd(protocol.VIDEO_START_CMD)
        for index, body in enumerate((b'AAAA', b'BBBB', b'CCCC')):
            self.fake.send_video(0, index, body)
        got = b''

        def collect():
            nonlocal got
            got += stream.read(1000)
            return got == b'AAAABBBBCCCC'
        wait_until(collect, what='the video bytes to come out of the stream')
