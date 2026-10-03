"""__send_command()'s on_response callback and __send_recv_command(), checked directly.

Both are private, so these are detail tests (see docs/interfaces_and_tests.md):
they reach into Tello's internals rather than going through a public method,
since no public method uses __send_recv_command() yet.
"""
import threading
import time

from tellopy import CommandSample
from tellopy._internal.error import TelloError

from tests.support import wire
from tests.support.harness import DroneTestCase, wait_until


class SendRecvCommandTest(DroneTestCase):

    def test_succeeds_on_the_first_ack(self):
        drone = self.connect()
        sample = drone._Tello__send_recv_command(wire.TAKEOFF_CMD, 'takeoff')
        self.assertIsInstance(sample, CommandSample)
        self.assertTrue(sample.acked)
        self.assertEqual(sample.name, 'takeoff')

    def test_retries_and_succeeds_once_the_drone_starts_acking(self):
        drone = self.start_drone(command_ack_timeout=0.2)
        self.connect()
        self.fake.ack_enabled = False
        result = {}

        def call():
            result['sample'] = drone._Tello__send_recv_command(wire.TAKEOFF_CMD, 'takeoff', retries=3)
        thread = threading.Thread(target=call)
        thread.start()

        self.fake.wait_for_cmd(wire.TAKEOFF_CMD, count=1)
        self.fake.ack_enabled = True    # only the *next* attempt will get acked

        def second_attempt_arrived():
            self.fake.send_flight_data()     # eviction of the first, unacked attempt is packet-driven
            return len(self.fake.received_cmds(wire.TAKEOFF_CMD)) >= 2
        wait_until(second_attempt_arrived, what='the retried takeoff')

        thread.join(5.0)
        self.assertFalse(thread.is_alive())
        self.assertTrue(result['sample'].acked)
        self.assertGreaterEqual(len(self.fake.received_cmds(wire.TAKEOFF_CMD)), 2)

    def test_raises_after_exhausting_every_retry(self):
        drone = self.start_drone(command_ack_timeout=0.2)
        self.connect()
        self.fake.ack_enabled = False
        result = {}

        def call():
            try:
                drone._Tello__send_recv_command(wire.TAKEOFF_CMD, 'takeoff', retries=2)
            except TelloError as ex:
                result['error'] = ex
        thread = threading.Thread(target=call)
        thread.start()

        def gave_up():
            self.fake.send_flight_data()
            return 'error' in result
        wait_until(gave_up, timeout=3.0, what='TelloError after exhausting retries')

        thread.join(5.0)
        self.assertFalse(thread.is_alive())
        self.assertIn('takeoff', str(result['error']))
        self.assertGreaterEqual(len(self.fake.received_cmds(wire.TAKEOFF_CMD)), 2)

    def test_a_reused_seq_settles_the_older_entry_as_a_timeout_first(self):
        """(cmd, seq_num) collisions can't be produced by 65536 real sends in
        a test; this engineers one directly to check __send_command() doesn't
        silently strand the older entry's on_response when its slot is reused."""
        drone = self.connect()
        seq_num = 0x1234
        drone.pkt_seq_num = seq_num - 1          # so __next_seq_num() hands out seq_num next
        stale_sample = CommandSample(wire.TAKEOFF_CMD, seq_num, 'stale_op', time.monotonic(), b'')
        settled = []
        drone._Tello__pending_sends[(wire.TAKEOFF_CMD, seq_num)] = (stale_sample, settled.append)

        ok = drone._Tello__send_command(wire.TAKEOFF_CMD, 'takeoff')

        self.assertTrue(ok)
        self.assertEqual(settled, [stale_sample])
        self.assertFalse(stale_sample.acked)
        self.fake.wait_for_cmd(wire.TAKEOFF_CMD)
