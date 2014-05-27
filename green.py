#!/usr/bin/python
"""
  Ultra slow conversion to GREEN pixels (exercise to do it in C instead)

       ./green.py <image or video file> [<PaVE frame number>|<src_cv2_ refrence log file>]
"""
import sys
import cv2

from airrace import main as imgmain # image debugging TODO move to launcher

def processGreen( frame, debug=False ):
  for y in xrange(frame.shape[0]):
    for x in xrange(frame.shape[1]):
      b,g,r = frame[y][x]
      if g > 10 and g > 1.1*r and g > 1.1*b:
        frame[y][x] = (0, 255, 0)
  cv2.imshow('image', frame)

if __name__ == "__main__":
  imgmain( sys.argv, processGreen )

