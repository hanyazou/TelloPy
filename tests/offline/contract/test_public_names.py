"""The names the package shows outside."""
import unittest

import tellopy


class PublicNamesTest(unittest.TestCase):

    def test_every_name_listed_is_there(self):
        for name in tellopy.__all__:
            self.assertTrue(hasattr(tellopy, name), name)
