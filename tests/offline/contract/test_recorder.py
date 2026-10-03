"""Recorder: recording what a drone publishes and what Containers make, and reading it back."""
import contextlib
import os
import tempfile
import unittest

from tellopy import (Container, FlightData, ImuContainer, LogImuAtti, Record, Recorder, Retimer, Sample, TickClock,
                     Tello)

from tests.support.fake_drone import FakeDrone, log_record
from tests.support.harness import DroneTestCase, free_udp_port, wait_until
from tests.support.synthetic import Log, one_of_each_sample

STARTED = 'tellopy.Recorder start recording'
STOPPED = 'tellopy.Recorder stop recording'


class TemporaryFile(object):
    def make_path(self, name='flight.jsonl'):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        return os.path.join(directory.name, name)


def kinds(recorder, *kinds_wanted):
    return [r for r in recorder.read() if r.kind in kinds_wanted]


class RecorderOfSamplesTest(TemporaryFile, unittest.TestCase):

    def test_every_kind_of_sample_comes_back_as_it_went(self):
        path, log = self.make_path(), Log()
        container = Container()
        recorder = Recorder(path, sources=[container], log=log)
        recorder.start()
        samples = one_of_each_sample()
        for sample in samples:
            container.add(sample)
        recorder.close()
        back = [r.item for r in recorder.read() if r.kind == 'container']
        self.assertEqual(len(back), len(samples))
        for original, restored in zip(samples, back):
            self.assertIs(type(restored), type(original))
            for name, value in vars(original).items():
                if name == 'log':
                    self.assertIs(restored.log, log)                # a record's log is the recorder's
                else:
                    self.assertEqual(getattr(restored, name), value, (type(original).__name__, name))
            if type(original).__str__ is not object.__str__:
                self.assertEqual(str(restored), str(original))
        self.assertEqual(log.errors, [])

    def test_nothing_is_written_until_it_is_started_and_the_file_is_not_touched_before(self):
        path = self.make_path()
        container = Container()
        recorder = Recorder(path, sources=[container])
        container.add(Sample(1.0))
        self.assertFalse(os.path.exists(path))
        recorder.start()
        container.add(Sample(2.0))
        recorder.close()
        self.assertEqual([r.item.event_time for r in kinds(recorder, 'container')], [2.0])

    def test_a_recording_goes_start_stop_start_with_notes_and_the_header(self):
        path = self.make_path()
        container = Container()
        recorder = Recorder(path)
        recorder.add(container)
        recorder.start()
        container.add(Sample(1.0))
        recorder.stop()
        container.add(Sample(2.0))                      # paused
        recorder.note('the second part')
        recorder.start()
        container.add(Sample(3.0))
        recorder.close()
        records = list(recorder.read())
        self.assertEqual(records[0].kind, 'header')
        self.assertEqual(records[0].item['version'], 1)
        self.assertEqual([(r.kind, r.item if r.kind == 'note' else r.item.event_time) for r in records[1:]],
                         [('note', STARTED), ('container', 1.0), ('note', STOPPED), ('note', 'the second part'),
                          ('note', STARTED), ('container', 3.0), ('note', STOPPED)])

    def test_a_record_says_when_it_was_written(self):
        path = self.make_path()
        recorder = Recorder(path)
        recorder.note('one')
        recorder.note('two')
        recorder.close()
        one, two = [r for r in recorder.read() if r.kind == 'note']
        self.assertIsInstance(one, Record)
        self.assertEqual((one.name, one.recv_time), ('', None))
        self.assertLessEqual(one.time, two.time)

    def test_containers_are_told_apart_by_their_names_and_the_second_of_a_name_is_numbered(self):
        path = self.make_path()
        raw, other, third = ImuContainer(), ImuContainer(), ImuContainer(name='mine')
        retimed = Retimer(raw, TickClock(raw))
        with Recorder(path, sources=[raw, other, third, retimed]) as recorder:
            for container, t in ((raw, 1.0), (other, 2.0), (third, 3.0)):
                sample = LogImuAtti()
                sample.tick, sample.recv_time, sample.event_time = 1, t, t
                container.add(sample)
        self.assertEqual(sorted((r.name, r.item.event_time) for r in kinds(recorder, 'container')),
                         [('ImuContainer', 1.0), ('ImuContainer_2', 2.0), ('Retimer(ImuContainer)', 1.0), ('mine', 3.0)])

    def test_a_container_added_twice_is_recorded_once(self):
        path = self.make_path()
        container = Container()
        with Recorder(path, sources=[container]) as recorder:
            recorder.add(container)
            container.add(Sample(1.0))
        self.assertEqual(len(kinds(recorder, 'container')), 1)

    def test_a_recording_is_added_to_and_not_emptied(self):
        path = self.make_path()
        container = Container()
        for t in (1.0, 2.0):
            with Recorder(path, sources=[container]) as recorder:
                container.add(Sample(t))
        records = list(recorder.read())
        self.assertEqual([r.kind for r in records if r.kind == 'header'], ['header', 'header'])
        self.assertEqual([r.item.event_time for r in records if r.kind == 'container'], [1.0, 2.0])

    def test_a_path_that_cannot_be_written_fails_at_start(self):
        recorder = Recorder(os.path.join(self.make_path('a'), 'no', 'such', 'directory', 'x.jsonl'))
        with self.assertRaises(OSError):
            recorder.start()

    def test_a_closed_recorder_cannot_be_used_again(self):
        recorder = Recorder(self.make_path())
        recorder.start()
        recorder.close()
        recorder.close()                                # closing again is harmless
        for use in (recorder.start, lambda: recorder.note('x'), lambda: recorder.add(Container())):
            with self.assertRaises(ValueError):
                use()

    def test_a_container_or_a_list_or_tuple_of_them_can_be_added_in_any_of_these_ways(self):
        first, second, third, fourth, fifth = Container(), Container(), Container(), Container(), Container()
        with Recorder(self.make_path(), sources=first) as recorder:     # one
            recorder.add(second, third)                                 # several
            recorder.add([fourth], (fifth,))                            # a list and a tuple
            for k, container in enumerate((first, second, third, fourth, fifth)):
                container.add(Sample(float(k)))
        self.assertEqual(sorted(r.item.event_time for r in kinds(recorder, 'container')), [0.0, 1.0, 2.0, 3.0, 4.0])
        with Recorder(self.make_path(), sources=(first, second)) as recorder:       # a tuple in the constructor
            first.add(Sample(9.0))
        self.assertEqual([r.item.event_time for r in kinds(recorder, 'container')], [9.0])

    def test_only_a_drone_a_container_or_an_estimator_can_be_added(self):
        recorder = Recorder(self.make_path())
        container = Container()
        with self.assertRaises(TypeError):
            recorder.add('a file name')
        with self.assertRaises(TypeError):
            recorder.add(Tello.EVENT_WIFI)              # an event is left out with exclude(), not added
        with self.assertRaises(TypeError):
            recorder.add(container, [container, 3])     # nothing is added when one of them is wrong
        with self.assertRaises(TypeError):
            recorder.add([[container]])                 # a list in a list is not a list of sources
        with recorder:
            container.add(Sample(1.0))
        self.assertEqual(kinds(recorder, 'container'), [])
        with self.assertRaises(ValueError):
            Recorder(self.make_path()).exclude('not an event')

    def test_a_value_that_is_not_json_is_written_as_its_repr_and_complained_of_once(self):
        path, log = self.make_path(), Log()
        container = Container()
        with Recorder(path, sources=[container], log=log) as recorder:
            for t in (1.0, 2.0):
                sample = Sample(t)
                sample.thing = object()
                container.add(sample)
        things = [r.item.thing for r in kinds(recorder, 'container')]
        self.assertEqual(len(things), 2)
        self.assertTrue(all(thing.startswith('<object object') for thing in things))
        self.assertEqual(len(log.errors), 1)
        self.assertIn('object', log.errors[0])

    def test_a_line_cut_off_at_the_end_is_passed_over_and_a_bad_one_elsewhere_is_not(self):
        path = self.make_path()
        container = Container()
        with Recorder(path, sources=[container]) as recorder:
            container.add(Sample(1.0))
        with open(path, 'a') as f:
            f.write('{"kind": "cont')
        self.assertEqual(len(kinds(recorder, 'container')), 1)
        with open(path, 'a') as f:
            f.write('ainer"}\n')                        # now the bad line is not the last one
            f.write('{"kind": "note", "name": "", "time": 0, "recv_time": null, "item": "x"}\n')
        with self.assertRaises(ValueError):
            list(recorder.read())

    def test_a_recording_of_another_version_is_refused(self):
        path = self.make_path()
        with open(path, 'w') as f:
            f.write('{"kind": "header", "name": "", "time": 0, "recv_time": null, "item": {"version": 99}}\n')
        with self.assertRaises(ValueError):
            list(Recorder(path).read())


class RecorderOfADroneTest(TemporaryFile, DroneTestCase):

    def flight(self, recorder, drone):
        """Let the drone say some things, and wait until the recorder has them."""
        self.fake.send_flight_data(battery=42)
        self.fake.send_wifi()
        self.fake.send_light()
        self.fake.send_log_data(log_record(2048, 1000, bytes(120)))
        wait_until(lambda: {drone.EVENT_FLIGHT_DATA.name, drone.EVENT_WIFI.name, drone.EVENT_SAMPLE_IMU.name} <=
                   set(r.name for r in kinds(recorder, 'event')), what='the events to be recorded')

    def test_the_events_of_a_drone_are_recorded_with_when_they_arrived(self):
        drone = self.start_drone()
        recorder = Recorder(self.make_path(), sources=[drone])
        recorder.start()
        self.connect()
        self.flight(recorder, drone)
        recorder.stop()
        events = dict((r.name, r) for r in kinds(recorder, 'event'))
        flight_data = events[drone.EVENT_FLIGHT_DATA.name]
        self.assertEqual(flight_data.drone, drone.name)
        self.assertIsInstance(flight_data.item, FlightData)
        self.assertEqual(flight_data.item.battery_percentage, 42)
        self.assertEqual(flight_data.recv_time, flight_data.item.recv_time)
        self.assertIsInstance(events[drone.EVENT_WIFI.name].item, bytes)
        self.assertIsInstance(events[drone.EVENT_SAMPLE_IMU.name].item, LogImuAtti)
        self.assertEqual(events[drone.EVENT_SAMPLE_IMU.name].item.tick, 1000)
        self.assertIsInstance(events[drone.EVENT_LOG_DATA.name].item, dict)         # not a Sample: its fields
        self.assertIn('records', events[drone.EVENT_LOG_DATA.name].item)

    def test_events_can_be_left_out_when_the_recorder_is_made_or_later(self):
        drone = self.start_drone()
        recorder = Recorder(self.make_path(), sources=drone, exclude=drone.EVENT_WIFI)
        recorder.start()
        recorder.exclude([drone.EVENT_LIGHT])           # after it has started
        self.connect()
        self.fake.send_wifi()
        self.fake.send_light()
        self.fake.send_flight_data(battery=1)
        wait_until(lambda: drone.EVENT_FLIGHT_DATA.name in set(r.name for r in kinds(recorder, 'event')), what='flight data')
        recorder.stop()
        names = set(r.name for r in kinds(recorder, 'event'))
        self.assertNotIn(drone.EVENT_WIFI.name, names)
        self.assertNotIn(drone.EVENT_LIGHT.name, names)

    def test_nothing_is_recorded_while_it_is_stopped(self):
        drone = self.start_drone()
        recorder = Recorder(self.make_path(), sources=[drone], exclude=[drone.EVENT_LOG_DATA])
        arrived = []                                    # what the drone has taken in, whether it is recorded or not
        drone.subscribe(drone.EVENT_FLIGHT_DATA, lambda event, sender, data: arrived.append(data.battery_percentage))
        self.connect()

        def say(battery):
            self.fake.send_flight_data(battery=battery)
            wait_until(lambda: battery in arrived, what='flight data %d' % battery)

        def recorded():
            return [r.item.battery_percentage for r in kinds(recorder, 'event') if r.name == drone.EVENT_FLIGHT_DATA.name]
        say(1)                                          # before it is started
        recorder.start()
        say(2)
        recorder.stop()
        say(3)                                          # while it is paused
        say(4)
        recorder.start()
        say(5)
        recorder.close()
        say(6)                                          # after it is closed
        self.assertEqual(recorded(), [2, 5])

    @contextlib.contextmanager
    def second_drone(self, **options):
        """Another Tello, connected to a fake drone of its own, for the length of a with block."""
        video_port = free_udp_port()
        fake = FakeDrone(video_port)
        drone = Tello(port=free_udp_port(), video_port=video_port, **options)
        drone.tello_addr = fake.address
        try:
            drone.connect()
            drone.wait_for_connection(5.0)
            yield drone, fake
        finally:
            drone.quit()
            fake.poke(drone.port)
            fake.stop()

    def batteries(self, recorder, drone_name=None):
        return [r.item.battery_percentage for r in kinds(recorder, 'event')
                if r.name == Tello.EVENT_FLIGHT_DATA.name and (drone_name is None or r.drone == drone_name)]

    def test_the_events_of_a_drone_that_was_not_added_are_not_recorded(self):
        drone = self.start_drone()
        recorder = Recorder(self.make_path(), sources=drone)
        recorder.start()
        self.connect()
        with self.second_drone() as (other, other_fake):
            other_fake.send_flight_data(battery=77)
            self.fake.send_flight_data(battery=11)
            wait_until(lambda: 11 in self.batteries(recorder), what='the first drone\'s flight data')
            other_fake.wait_for_cmd(0x0050)
        recorder.close()
        self.assertNotIn(77, self.batteries(recorder))

    def test_a_record_of_an_event_says_which_drone_it_came_from_and_drones_with_one_name_are_numbered(self):
        drone = self.start_drone()
        recorder = Recorder(self.make_path(), sources=drone)
        self.connect()
        with self.second_drone() as (other, other_fake), self.second_drone(name='the third') as (third, third_fake):
            recorder.add([other, third])                    # a Tello with no name given is called Tello, as the first
            recorder.start()
            self.fake.send_flight_data(battery=1)
            other_fake.send_flight_data(battery=2)
            third_fake.send_flight_data(battery=3)
            wait_until(lambda: len(self.batteries(recorder)) >= 3, what='flight data of the three')
        recorder.close()
        self.assertEqual(self.batteries(recorder, 'Tello'), [1])
        self.assertEqual(self.batteries(recorder, 'Tello_2'), [2])
        self.assertEqual(self.batteries(recorder, 'the third'), [3])
        self.assertTrue(all(r.drone is None for r in recorder.read() if r.kind != 'event'))
