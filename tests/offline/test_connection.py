"""Connecting, commands and their acknowledgements, video, and quitting,
against a FakeDrone."""
import time

from tellopy._internal import protocol

from .fake_drone import log_record
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

    def test_command_is_sent_intact_acked_and_reported(self):
        drone = self.start_drone()
        acks = []
        drone.subscribe(drone.EVENT_SAMPLE_COMMAND_ACK, lambda event, sender, data: acks.append(data))
        self.connect()
        drone.takeoff()
        wait_until(lambda: acks, what='the ack of takeoff')

        sent = self.fake.wait_for_cmd(protocol.TAKEOFF_CMD)[0]
        self.assertTrue(sent.crc_ok)
        self.assertNotEqual(sent.seq, 0)
        sample = acks[0]
        self.assertEqual((sample.name, sample.cmd, sample.seq), ('takeoff', protocol.TAKEOFF_CMD, sent.seq))
        self.assertTrue(sample.acked)
        self.assertTrue(0 <= sample.rtt < 1.0)
        self.assertEqual(sample.event_time, sample.send_time)

    def test_unanswered_command_is_reported_as_timed_out(self):
        drone = self.start_drone()
        timeouts, acks = [], []
        drone.subscribe(drone.EVENT_SAMPLE_COMMAND_TIMEOUT, lambda event, sender, data: timeouts.append(data))
        drone.subscribe(drone.EVENT_SAMPLE_COMMAND_ACK, lambda event, sender, data: acks.append(data))
        self.connect()
        self.fake.wait_for_cmd(protocol.TIME_CMD)
        wait_until(lambda: not drone._Tello__pending_sends, what='replies to the startup commands')
        self.fake.ack_enabled = False
        drone.COMMAND_ACK_TIMEOUT_SEC = 0.2
        drone.land()

        def keep_the_check_running():
            self.fake.send_flight_data()
            return timeouts
        wait_until(keep_the_check_running, what='the timeout of land')
        self.assertEqual(timeouts[0].name, 'land')
        self.assertFalse(timeouts[0].acked)
        self.assertEqual([a.name for a in acks if a.name == 'land'], [])

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


class LogRecordTest(DroneTestCase):

    def test_each_record_is_published_on_its_own(self):
        drone = self.start_drone()
        imus, tofs, raws, messages = [], [], [], []
        drone.subscribe(drone.EVENT_SAMPLE_IMU, lambda event, sender, data: imus.append(data))
        drone.subscribe(drone.EVENT_SAMPLE_TOF, lambda event, sender, data: tofs.append(data))
        drone.subscribe(drone.EVENT_SAMPLE_RAW, lambda event, sender, data: raws.append(data))
        drone.subscribe(drone.EVENT_LOG_DATA, lambda event, sender, data: messages.append(data))
        self.connect()
        self.fake.send_log_data(
            log_record(2048, 100, bytes(120)),
            log_record(2048, 200, bytes(120)),
            log_record(16, 150, bytes([0x64, 0, 1, 7])),
            log_record(4242, 300, b'abc'))
        wait_until(lambda: raws, what='the whole message to be processed')

        self.assertEqual([s.tick for s in imus], [100, 200])
        self.assertIsNot(imus[0], imus[1])
        self.assertEqual((tofs[0].distance, tofs[0].flag, tofs[0].counter), (100, 1, 7))
        self.assertEqual((raws[0].record_id, raws[0].payload), (4242, b'abc'))
        self.assertTrue(all(s.event_time == s.recv_time for s in imus + tofs + raws))
        # the per-message event still fires once, with the latest IMU record
        self.assertEqual(len(messages), 1)
        self.assertEqual(messages[0].imu.tick, 200)

    def test_a_record_too_short_to_decode_is_skipped_alone(self):
        drone = self.start_drone()
        imus, tofs = [], []
        drone.subscribe(drone.EVENT_SAMPLE_IMU, lambda event, sender, data: imus.append(data))
        drone.subscribe(drone.EVENT_SAMPLE_TOF, lambda event, sender, data: tofs.append(data))
        self.connect()
        self.fake.send_log_data(
            log_record(2048, 100, bytes(10)),           # far shorter than an IMU record
            log_record(16, 101, bytes([0x64, 0, 1, 7])))
        wait_until(lambda: tofs, what='the record after the bad one')
        self.assertEqual(imus, [])


class StickTest(DroneTestCase):

    def test_every_stick_command_sent_is_published_with_what_was_sent(self):
        drone = self.start_drone()
        sticks = []
        drone.subscribe(drone.EVENT_SAMPLE_STICK, lambda event, sender, data: sticks.append(data))
        self.connect()
        drone.counter_clockwise(50)         # yaw -0.5
        drone.up(20)                        # throttle 0.2
        for _ in range(20):                 # a stick command goes out per packet received
            self.fake.send_flight_data()
            time.sleep(0.01)
        wait_until(lambda: len(sticks) >= 10, what='stick samples')

        last = sticks[-1]
        self.assertEqual((last.roll, last.pitch, last.throttle, last.yaw, last.fast_mode),
                         (0.0, 0.0, 0.2, -0.5, False))
        times = [s.event_time for s in sticks]
        self.assertEqual(times, sorted(times))
        self.assertTrue(all(s.tick is None and s.recv_time is None for s in sticks))
        # each one published matches one packet that reached the drone
        on_wire = self.fake.received_cmds(protocol.STICK_CMD)
        self.assertLessEqual(abs(len(on_wire) - len(sticks)), 2)
