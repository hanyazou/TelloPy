"""The rules of docs/interfaces_and_tests.md that can be checked, checked.

A contract test uses only the package's public names and tests/support, and
every name the package shows outside is used by some contract test.
"""
import ast
import glob
import inspect
import os
import textwrap
import unittest

import tellopy

CONTRACT_TESTS = sorted(glob.glob(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'contract', 'test_*.py')))

# Classes the library had before the timing work; the rules on names do not ask for their names to be tested.
EXISTING_CLASSES = {'Tello', 'Logger', 'VideoStream', 'FlightData', 'LogData', 'LogImuAtti', 'LogNewMvoFeedback'}
EXISTING_NAMES = {'update', 'add_note', 'with_traceback'}   # the latter two are BaseException's, not the package's
# What a subclass declares; a test of the subclass does not have to mention it.
DECLARATIONS = {'EVENTS', 'SAMPLE', 'ID'}


def parse(path):
    with open(path) as f:
        return ast.parse(f.read(), path)


def mentioned_in_contract_tests():
    """Every name a contract test uses: as a name, as an attribute or as a keyword argument."""
    names = set()
    for path in CONTRACT_TESTS:
        for node in ast.walk(parse(path)):
            if isinstance(node, ast.Name):
                names.add(node.id)
            elif isinstance(node, ast.Attribute):
                names.add(node.attr)
            elif isinstance(node, ast.keyword) and node.arg:
                names.add(node.arg)
    return names


def public_classes():
    return [(name, getattr(tellopy, name)) for name in tellopy.__all__ if inspect.isclass(getattr(tellopy, name))]


def data_fields(cls):
    """The public names a class gives its instances, as `self.name = ...` in its own methods."""
    if hasattr(cls, '_fields'):         # a named tuple
        return set(cls._fields)
    fields = set()
    for klass in cls.__mro__:
        if klass.__module__.startswith('tellopy'):
            for node in ast.walk(ast.parse(textwrap.dedent(inspect.getsource(klass)))):
                if (isinstance(node, ast.Attribute) and isinstance(node.ctx, ast.Store)
                        and isinstance(node.value, ast.Name) and node.value.id == 'self' and not node.attr.startswith('_')):
                    fields.add(node.attr)
    return fields


class ContractTestsUseOnlyPublicNamesTest(unittest.TestCase):

    def test_they_do_not_import_from_the_internals(self):
        for path in CONTRACT_TESTS:
            for node in ast.walk(parse(path)):
                if isinstance(node, ast.ImportFrom):
                    self.assertFalse((node.module or '').startswith('tellopy._internal'), (path, node.module))
                elif isinstance(node, ast.Import):
                    for alias in node.names:
                        self.assertFalse(alias.name.startswith('tellopy._internal'), (path, alias.name))

    def test_they_do_not_touch_private_names(self):
        for path in CONTRACT_TESTS:
            for node in ast.walk(parse(path)):
                if isinstance(node, ast.Attribute) and node.attr.startswith('_') and not node.attr.startswith('__'):
                    on_self = isinstance(node.value, ast.Name) and node.value.id == 'self'
                    self.assertTrue(on_self, '%s:%d touches %s' % (path, node.lineno, node.attr))


class EveryPublicNameIsUsedTest(unittest.TestCase):

    def test_the_classes_and_what_they_offer_are_used_by_a_contract_test(self):
        used = mentioned_in_contract_tests()
        missing = []
        for name, cls in public_classes():
            if name in EXISTING_CLASSES:
                continue
            if name not in used:
                missing.append(name)
            for member, _ in inspect.getmembers(cls):
                if (not member.startswith('_') and member not in dir(object) and member not in DECLARATIONS
                        and member not in EXISTING_NAMES and member not in used):
                    missing.append('%s.%s' % (name, member))
        self.assertEqual(missing, [], 'no contract test uses these: they need one, or are not to be public')

    def test_the_fields_of_the_records_are_used_by_a_contract_test(self):
        used = mentioned_in_contract_tests()
        missing = []
        for name, cls in public_classes():
            if name in EXISTING_CLASSES:
                continue
            for field in sorted(data_fields(cls)):
                if field not in used and field not in EXISTING_NAMES and field not in DECLARATIONS:
                    missing.append('%s.%s' % (name, field))
        self.assertEqual(missing, [], 'no contract test uses these: they need one, or are not to be public')
