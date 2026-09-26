"""What Tello puts on the wire, checked packet by packet."""
import time

from tests.support import wire
from tests.support.harness import DroneTestCase


class PacketDetailTest(DroneTestCase):

    def test_every_packet_sent_has_a_valid_crc_and_a_distinct_seq(self):
        drone = self.connect()
        for _ in range(20):
            drone.land()
        # let stick commands go out too: one is sent per packet received
        for _ in range(10):
            self.fake.send_flight_data()
            time.sleep(0.01)
        self.fake.wait_for_cmd(wire.LAND_CMD, count=20)
        self.fake.wait_for_cmd(wire.STICK_CMD, count=3)
        with self.fake.lock:
            received = list(self.fake.received)
        self.assertTrue(all(r.crc_ok for r in received))
        seqs = [r.seq for r in received]
        self.assertNotIn(0, seqs)
        self.assertEqual(len(seqs), len(set(seqs)), 'a seq number was used twice')
