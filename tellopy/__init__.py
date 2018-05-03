"""
DJI Tello controller

This is a python package which controlls DJI toy drone 'Tello'. The major portion of the source
code was ported from the driver of GOBOT project. Please refer their blog post at
https://gobot.io/blog/2018/04/20/hello-tello-hacking-drones-with-go
"""
from tellopy._internal.tello import Tello

__all__ = ["Tello"]
