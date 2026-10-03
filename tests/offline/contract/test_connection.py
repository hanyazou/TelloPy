"""A real Tello against a FakeDrone: connecting, commands, video, telemetry and sticks."""
import time

from tellopy import CommandSample, FlightData, Sample

from tests.support import wire
from tests.support.fake_drone import log_record
from tests.support.harness import DroneTestCase, wait_until


class ConnectionTest(DroneTestCase):

    def test_connect_handshake_then_time_is_sent(self):
        self.connect()
        found = self.fake.wait_for_cmd(wire.TIME_CMD)
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

        sent = self.fake.wait_for_cmd(wire.TAKEOFF_CMD)[0]
        self.assertTrue(sent.crc_ok)
        self.assertNotEqual(sent.seq, 0)
        sample = acks[0]
        self.assertIsInstance(sample, CommandSample)
        self.assertEqual((sample.name, sample.cmd, sample.seq), ('takeoff', wire.TAKEOFF_CMD, sent.seq))
        self.assertEqual(sample.ack_payload, sent.payload)          # the fake drone echoes what it was sent
        self.assertTrue(sample.acked)
        self.assertTrue(0 <= sample.rtt < 1.0)
        self.assertEqual(sample.event_time, sample.send_time)

    def test_unanswered_command_is_reported_as_timed_out(self):
        drone = self.start_drone(command_ack_timeout=0.5)
        timeouts, acks = [], []
        drone.subscribe(drone.EVENT_SAMPLE_COMMAND_TIMEOUT, lambda event, sender, data: timeouts.append(data))
        drone.subscribe(drone.EVENT_SAMPLE_COMMAND_ACK, lambda event, sender, data: acks.append(data))
        self.connect()
        self.fake.wait_for_cmd(wire.TIME_CMD)
        self.fake.ack_enabled = False
        drone.land()

        def timeouts_of_land():
            self.fake.send_flight_data()            # the check for timeouts runs as packets come in
            return [t for t in timeouts if t.name == 'land']
        wait_until(timeouts_of_land, what='the timeout of land')
        self.assertFalse(timeouts_of_land()[0].acked)
        self.assertEqual([a.name for a in acks if a.name == 'land'], [])


class VideoTest(DroneTestCase):

    def test_video_stream_delivers_the_bytes_sent(self):
        drone = self.connect()
        stream = drone.get_video_stream()
        self.fake.wait_for_cmd(wire.VIDEO_START_CMD)
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
        on_wire = self.fake.received_cmds(wire.STICK_CMD)
        self.assertLessEqual(abs(len(on_wire) - len(sticks)), 2)


class CalibrationTest(DroneTestCase):

    def test_the_calibration_status_says_how_far_it_has_come(self):
        drone = self.connect()
        got = []
        drone.subscribe(drone.EVENT_CALIBRATION_STATUS, lambda event, sender, data: got.append(data))
        drone.start_calibration()
        self.fake.send_calibration_status(step_mask=0b000111, progress=40)
        wait_until(lambda: got, what='a calibration status')
        status = got[0]
        self.assertEqual((status.step_mask, status.progress, status.steps_done, status.done), (0b000111, 40, 3, False))

    def test_the_calibration_is_done_when_the_progress_reaches_100(self):
        drone = self.connect()
        got = []
        drone.subscribe(drone.EVENT_CALIBRATION_STATUS, lambda event, sender, data: got.append(data))
        self.assertFalse(drone.calibration_active)
        drone.start_calibration()
        self.assertTrue(drone.calibration_active)
        with self.assertRaises(AttributeError):
            drone.calibration_active = False               # it says what the drone is doing; it is not a switch
        self.fake.send_calibration_status(step_mask=0x3f, progress=100)
        wait_until(lambda: got, what='a calibration status')
        status = got[0]
        self.assertEqual((status.steps_done, status.done), (6, True))
        wait_until(lambda: not drone.calibration_active, what='the calibration to end')


class NameTest(DroneTestCase):

    def test_a_drone_has_a_name_which_is_its_class_unless_it_is_given_one(self):
        drone = self.start_drone()
        self.assertEqual(drone.name, 'Tello')
        with self.assertRaises(AttributeError):
            drone.name = 'renamed'                      # given at construction, not changed after

    def test_a_drone_can_be_given_a_name(self):
        self.assertEqual(self.start_drone(name='the other').name, 'the other')


class FlightDataTest(DroneTestCase):

    def test_flight_data_is_a_sample_stamped_with_when_it_arrived(self):
        drone = self.connect()
        got = []
        drone.subscribe(drone.EVENT_FLIGHT_DATA, lambda event, sender, data, recv_time: got.append((data, recv_time)))
        self.fake.send_flight_data(battery=55)
        wait_until(lambda: got, what='flight data')
        data, recv_time = got[0]
        self.assertIsInstance(data, FlightData)
        self.assertIsInstance(data, Sample)
        self.assertEqual(data.battery_percentage, 55)
        self.assertEqual((data.recv_time, data.event_time), (recv_time, recv_time))
        self.assertIsNone(data.tick)
        self.assertIn('BAT: 55', str(data))
