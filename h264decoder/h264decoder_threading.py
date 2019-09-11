#!/usr/bin/env python2

import os
import sys
import numpy as np
import threading
import libh264decoder

thefile = 'testclip.h264'

class DecoderThread(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    self.counter = 0
  def run(self):
    with open(thefile,'r') as f:
      num_frames = 0
      decoder = libh264decoder.H264Decoder()
      # Original way is 60 fps on laptop, this way is 100 fps
      while 1:
        data_in = f.read(1024)
        if not data_in:
          break
        framelist = decoder.decode(data_in)
        for frame in framelist:
          print 'thread %s decoded frame %i' % (self.ident, self.counter)
          self.counter += 1
      return num_frames

a = DecoderThread()
b = DecoderThread()
a.start()
b.start()
a.join()
b.join()