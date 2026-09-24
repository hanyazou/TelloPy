import unittest

from tellopy._internal.container import Container, ImuContainer, StickContainer
from tellopy._internal.sample import Sample

from .fake_drone import log_record
from .harness import DroneTestCase, wait_until


def sample(t):
    return Sample(event_time=t)


class ContainerTest(unittest.TestCase):

    def test_old_samples_are_dropped_by_age_measured_from_the_newest(self):
        container = Container(max_age=1.0)
        for t in (0.0, 0.5, 1.0, 1.4, 2.0):
            container.add(sample(t))
        self.assertEqual([s.event_time for s in container], [1.0, 1.4, 2.0])
        self.assertEqual(container.count, 5)            # how many ever came in

    def test_count_limit(self):
        container = Container(max_age=1e9, max_count=3)
        for t in range(10):
            container.add(sample(float(t)))
        self.assertEqual([s.event_time for s in container], [7.0, 8.0, 9.0])
        self.assertEqual(len(container), 3)

    def test_latest_and_window(self):
        container = Container(max_age=1e9)
        self.assertIsNone(container.latest())
        self.assertEqual(container.window(0, 10), [])
        for t in (1.0, 2.0, 3.0, 4.0, 5.0):
            container.add(sample(t))
        self.assertEqual(container.latest().event_time, 5.0)
        self.assertEqual([s.event_time for s in container.window(2.0, 4.0)], [2.0, 3.0, 4.0])
        self.assertEqual([s.event_time for s in container.window(4.5, 99)], [5.0])
        self.assertEqual(container.window(6, 7), [])

    def test_listeners_hear_every_sample_and_may_look_at_the_container(self):
        container = Container()
        heard = []
        container.subscribe(lambda s: heard.append((s.event_time, len(container), container.latest() is s)))
        container.add(sample(1.0))
        container.add(sample(2.0))
        self.assertEqual(heard, [(1.0, 1, True), (2.0, 2, True)])

    def test_unsubscribed_listener_hears_no_more(self):
        container = Container()
        heard = []
        container.subscribe(heard.append)
        container.add(sample(1.0))
        container.unsubscribe(heard.append)
        container.add(sample(2.0))
        self.assertEqual(len(heard), 1)

    def test_a_container_refuses_samples_of_the_wrong_kind(self):
        with self.assertRaises(TypeError):
            ImuContainer().add(sample(1.0))


class ContainerOnADroneTest(DroneTestCase):

    def test_fills_from_the_drones_events_until_closed(self):
        drone = self.start_drone()
        imus = ImuContainer(drone)
        sticks = StickContainer(drone)
        self.connect()
        for tick in (100, 200, 300):
            self.fake.send_log_data(log_record(2048, tick, bytes(120)))
        wait_until(lambda: len(imus) == 3, what='three IMU records')
        self.assertEqual([s.tick for s in imus], [100, 200, 300])
        self.assertEqual(imus.latest().tick, 300)
        wait_until(lambda: len(sticks) > 0, what='some stick samples')

        imus.close()
        self.fake.send_log_data(log_record(2048, 400, bytes(120)))
        self.fake.send_flight_data()
        wait_until(lambda: self.fake.received_cmds(0x0050), what='the drone to still be talking')
        self.assertEqual(imus.count, 3)

    def test_two_drones_dont_mix(self):
        first = self.start_drone()
        imus = ImuContainer(first)
        self.connect()
        # a second drone's records must not end up in the first one's container
        from tellopy import Tello
        from .harness import free_udp_port
        from .fake_drone import FakeDrone
        other_video = free_udp_port()
        other_fake = FakeDrone(other_video)
        other = Tello(port=free_udp_port(), video_port=other_video)
        other.tello_addr = other_fake.address
        try:
            other.connect()
            other.wait_for_connection(5.0)
            other_fake.send_log_data(log_record(2048, 999, bytes(120)))
            self.fake.send_log_data(log_record(2048, 111, bytes(120)))
            wait_until(lambda: len(imus) == 1, what='the first drone\'s record')
            other_fake.wait_for_cmd(0x0050)                 # the other drone's traffic has been processed
            self.assertEqual([s.tick for s in imus], [111])
        finally:
            other.quit()
            other_fake.poke(other.port)
            other_fake.stop()
