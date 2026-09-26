"""Which log a Container uses."""
import unittest

from tellopy import Container, ImuContainer, Logger
from tellopy._internal.tello import log as library_log

from tests.support.harness import DroneTestCase


class ContainerLogTest(unittest.TestCase):

    def test_without_a_drone_or_a_log_it_uses_the_librarys_own(self):
        self.assertIs(Container().log, library_log)
        self.assertIs(ImuContainer().log, library_log)

    def test_a_log_that_is_given_is_used(self):
        mine = Logger('mine')
        self.assertIs(Container(log=mine).log, mine)
        self.assertIs(ImuContainer(log=mine).log, mine)


class ContainerOnADroneLogTest(DroneTestCase):

    def test_takes_the_drones_log_unless_given_one(self):
        drone = self.start_drone()
        self.assertIs(ImuContainer(drone).log, drone.log)
        drone.log = Logger('this drone')            # a drone with a log of its own
        self.assertIs(ImuContainer(drone).log, drone.log)
        mine = Logger('mine')
        self.assertIs(ImuContainer(drone, log=mine).log, mine)
