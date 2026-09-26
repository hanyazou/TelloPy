"""The names the package shows outside."""
import unittest

import tellopy


class PublicNamesTest(unittest.TestCase):

    def test_every_name_listed_is_there(self):
        for name in tellopy.__all__:
            self.assertTrue(hasattr(tellopy, name), name)

    def test_every_kind_of_sample_the_library_defines_is_public(self):
        public = {getattr(tellopy, name) for name in tellopy.__all__}

        def subclasses(cls):
            for sub in cls.__subclasses__():
                yield sub
                yield from subclasses(sub)
        for cls in subclasses(tellopy.Sample):
            if cls.__module__.startswith('tellopy'):
                self.assertIn(cls, public, cls.__name__)
