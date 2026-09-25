"""How the drone answers what the host tells it, measured on a real drone.

The scripts here fly it (or talk to it) and record what happened; they are
experiments, not part of the library or examples of using it, and are kept
as loose as experiments should be. The tests next to them run the scripts
against a FakeDrone (tests/offline/fake_drone.py), so they need no drone:
./tests/offline/test.sh runs them with the rest.
"""
