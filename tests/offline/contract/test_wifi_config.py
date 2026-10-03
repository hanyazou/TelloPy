"""tellopy.Tello.get_ssid()/set_ssid()/get_password()/set_password(): what their docstrings promise."""
import threading

import tellopy

from tests.support import wire
from tests.support.harness import DroneTestCase, wait_until


class SsidTest(DroneTestCase):

    def test_get_ssid_returns_the_current_value(self):
        drone = self.connect()
        self.fake.ack_enabled = False
        result = {}

        def call():
            result['ssid'] = drone.get_ssid()
        thread = threading.Thread(target=call)
        thread.start()

        sent = self.fake.wait_for_cmd(wire.SSID_MSG)[0]
        self.fake.send_ssid_reply('TELLO-AB419E', sent.seq)

        thread.join(5.0)
        self.assertFalse(thread.is_alive())
        self.assertEqual(result['ssid'], 'TELLO-AB419E')

    def test_get_ssid_raises_when_the_drone_never_answers(self):
        drone = self.start_drone(command_ack_timeout=0.2)
        self.connect()
        self.fake.ack_enabled = False
        result = {}

        def call():
            try:
                drone.get_ssid()
            except tellopy.TelloError as ex:
                result['error'] = ex
        thread = threading.Thread(target=call)
        thread.start()

        def gave_up():
            self.fake.send_flight_data()     # drives the retry/timeout machinery along
            return 'error' in result
        wait_until(gave_up, timeout=3.0, what='TelloError from get_ssid()')

        thread.join(5.0)
        self.assertFalse(thread.is_alive())
        self.assertIn('get_ssid', result['error'].msg)

    def test_set_ssid_succeeds_when_acked(self):
        drone = self.connect()
        drone.set_ssid('TELLO-AB419E')          # no exception raised means success

    def test_set_ssid_raises_when_the_drone_never_acks(self):
        drone = self.start_drone(command_ack_timeout=0.2)
        self.connect()
        self.fake.ack_enabled = False
        result = {}

        def call():
            try:
                drone.set_ssid('TELLO-AB419E')
            except tellopy.TelloError as ex:
                result['error'] = ex
        thread = threading.Thread(target=call)
        thread.start()

        def gave_up():
            self.fake.send_flight_data()
            return 'error' in result
        wait_until(gave_up, timeout=3.0, what='TelloError from set_ssid()')

        thread.join(5.0)
        self.assertFalse(thread.is_alive())


class PasswordTest(DroneTestCase):

    def test_get_password_returns_the_current_value(self):
        drone = self.connect()
        self.fake.ack_enabled = False
        result = {}

        def call():
            result['password'] = drone.get_password()
        thread = threading.Thread(target=call)
        thread.start()

        sent = self.fake.wait_for_cmd(wire.SSID_PASSWORD_MSG)[0]
        self.fake.send_password_reply('13536135', sent.seq)

        thread.join(5.0)
        self.assertFalse(thread.is_alive())
        self.assertEqual(result['password'], '13536135')

    def test_set_password_succeeds_when_acked(self):
        drone = self.connect()
        drone.set_password('13536135')

    def test_set_password_rejects_a_password_longer_than_19_bytes(self):
        drone = self.start_drone()
        with self.assertRaises(ValueError):
            drone.set_password('x' * 20)
