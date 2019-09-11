#!/usr/bin/env python2

import os
import sys
import numpy as np
import time

import libh264decoder

thefile = 'testclip.h264'

if 1:
  def conv((frame, w, h, ls)):
    frame = np.fromstring(frame, dtype = np.ubyte, count = len(frame), sep = '') # this conversion drops fps from 200 to 150
    frame = frame.reshape((h, ls/3, 3))
    frame = frame[:,:w,:]
else:
  def conv(frame):
    pass

def run_decode():
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
        conv(frame)
        num_frames += 1
    return num_frames

def run_decode_frame():
  with open(thefile,'r') as f:
    num_frames = 0
    decoder = libh264decoder.H264Decoder()
    # On laptop this way is 80 fps.
    while 1:
      data_in = f.read(1024)
      if not data_in:
        break
      while data_in:
        frame, nread = decoder.decode_frame(data_in)
        data_in = data_in[nread:]
        if frame[0]:
          conv(frame)
          num_frames += 1
  return num_frames

def measure(fun):
  t0 = time.time()
  num_frames = fun()
  t1 = time.time()
  print '%s fps = %f' % (fun.__name__, num_frames/(t1-t0))

run_decode()
measure(run_decode)
measure(run_decode_frame)
