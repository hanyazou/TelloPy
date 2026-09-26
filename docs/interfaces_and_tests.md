# Interfaces and Tests

How to read this document: these are the rules the code and the tests are written by.
They say what is done, not that nothing else may be done.
Code and tests written before these rules may not follow them yet.

## 1. What Is Interface and What Is Not

- A name is interface if the library's users are meant to rely on it.
  Everything else is internal.
- Two things show which is which: a leading underscore, and `tellopy/__init__.py`, which decides what is shown outside the package.
- `tellopy/__init__.py` is a whitelist: a class is public only if it is listed there.
  Nothing checks that a class that is not listed was left out on purpose.
- Programs that import `tellopy._internal` or use names with an underscore are not covered by compatibility.
- A name that existed before keeps working.
  Making it a read-only property is acceptable.
  Giving it an underscore is done only when it is very unlikely that anything outside uses it, and is judged case by case.
- A value shown outside is a read-only property.
  Users are not meant to assign to it, and assigning is not part of the interface.
- A value only the class itself needs is kept under an underscore name, whether it is given at construction or worked out later.
  That also keeps users from assigning to it.
- A value that only debugging or a test looks at is kept under an underscore name.
  If it is more than that, whether it belongs in a Sample is considered.
- What a Sample already carries is not also kept as an attribute of the object that made it.
- A setting that need not change while the program runs is a keyword argument of the constructor.
  A setting that gains from being changed while it runs gets a method, `set_xxx()`, as part of the interface.
- A helper used only inside one class gets a double underscore (`__foo`).
  A name that subclasses may use gets a single underscore (`_foo`), because a double one would break their calls.
- A value that a base class gives every subclass, with a clear meaning, is a read-only property, for example the number of Samples a Container has received.
- One value is not kept under two names, unless the two mean different things, as the time a command was sent and the time of the event it is.
- A plain record of values, a Sample or a decoded message such as CalibrationStatus, has ordinary attributes and they are not made read-only.
  The library makes a new Sample instead of changing one it has published.

## 2. Defining an Interface

- The definition of an interface is the docstring of the public class or method.
  There is no separate document for it.
- A docstring says what a user can rely on, in the user's terms: what goes in, what comes out, and what the reader can tell from what comes out.
  For example, a Retimer republishes a Sample whether or not the clock has an estimate, and the Sample it publishes tells which.
- A docstring does not restate the implementation: which value a field takes in which case, thresholds, and how something is worked out are left out.
- What a field means is defined once, in the docstring of the class that has it.
  Other docstrings refer to the concept by name.

## 3. Tests

- Tests are of two kinds, by what they check.
  A contract test checks what a docstring promises.
  A detail test checks specifics: particular values, thresholds, how something is worked out.
  Contract tests are in `tests/offline/contract` and detail tests in `tests/offline/detail`.
- A test may reach into internals.
  Adding a public name only so that a test can reach something is not done.
- A contract test uses only public names and the test support modules.
  The test support modules are in `tests/support`.
  They hold what builds Samples, synthetic data and stand-ins, and may reach into internals.
- Two checks in `tests/offline/test_interface_rules.py` keep to these rules.
  One is that a contract test imports only from the package and reaches no private name.
  The other is that every public class, its methods and properties, and the fields of its records are used by some contract test.
  Arguments of a constructor, what a subclass declares (`EVENTS`, `SAMPLE`, `ID`) and the classes the library had before the timing work are left out of the second.
- The components talk to one another only through Samples.
  A stand-in for an upstream component is a Container filled with Samples made by hand.
- A test has an answer that does not come from the code under test: the known truth of synthetic data, a reference computed independently, or a recorded flight replayed with the same output as before.
  Comparing internal state with what the code has just set says nothing about behaviour.
- A test is not written for a number that only a simple implementation happens to use, such as how many packets a clock discards at start-up.
  What is checked is the behaviour: that the estimate is not thrown off by the odd values seen at start-up.
