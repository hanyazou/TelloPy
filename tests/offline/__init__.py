"""Tests that need neither a drone nor Wi-Fi.

A FakeDrone (fake_drone.py) stands in for the drone on loopback UDP, and
the real Tello -- real sockets, real threads -- talks to it. Run them with

    ./tests/offline/test.sh

(The scripts directly under tests/ are the opposite: they fly a real drone.)
"""
