#!/bin/sh
# Run the offline tests -- those in tests/offline, and those of the scripts in
# tests/response, which run against a fake drone: no drone, no Wi-Fi, and
# nothing to install; they need only Python 3 and its standard library.
#
#   ./tests/offline/test.sh                       run them all
#   ./tests/offline/test.sh -v                    ... listing each test as it runs
#   ./tests/offline/test.sh tests.offline.test_connection.VideoTest
#                                                 run just the named test(s)
#
# Set PYTHON to use a particular interpreter (default: python3).
# Exits non-zero if any test fails.

cd "$(dirname "$0")/../.." || exit 1

python=${PYTHON:-python3}
if ! command -v "$python" >/dev/null 2>&1; then
    echo "test.sh: $python not found (set PYTHON=/path/to/python3)" >&2
    exit 127
fi

# An argument that isn't an option names tests to run; otherwise run them all.
if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then
    exec "$python" -m unittest "$@"
fi
exec "$python" -m unittest discover -s tests -t . -p 'test_*.py' "$@"
