#!/usr/bin/python
"""
  Ultra slow conversion to GREEN pixels (exercise to do it in C instead)

       ./green.py <image or video file> [<PaVE frame number>|<src_cv2_ refrence log file>]
"""
import sys
import cv2
import cimg

from airrace import main as imgmain # image debugging TODO move to launcher

def processGreenVer0( frame, debug=False ):
  for y in xrange(frame.shape[0]):
    for x in xrange(frame.shape[1]):
      b,g,r = frame[y][x]
      if g > 10 and g > 1.1*r and g > 1.1*b:
        frame[y][x] = (0, 255, 0)
  cv2.imshow('image', frame)

def processGreen( frame, debug=False ):
  cimg.green( frame, 1.1 )
  cv2.imshow('image', frame)

"""
  Avoid Green
  - for given rectangle where direction is defined by fromX to toX count green pixels until limit is reached
"""
def avoidGreen0( frame, fromX, toX, fromY, toY, limit ):
  # code to be realized in C
  count = 0
  step = 1 if fromX < toX else -1
  for x in xrange(fromX, toX, step):
    for y in xrange(fromY, toY):
      b,g,r = frame[y][x]
      if g > 10 and g > 1.1*r and g > 1.1*b:
        frame[y][x] = (0, 255, 0)
        count +=1
      else:
        frame[y][x] = (255, 255, 255)
    if count > limit:
      break
  return x

def avoidGreen( frame, fromX, toX, fromY, toY, limit ):
  "numpy implementation"
  return cimg.avoidGreen( frame, fromX, toX, fromY, toY, limit, 1.1 )

def stripLeftRight( frame, fromY, toY, limit ):
  width = frame.shape[1]
  xL = avoidGreen( frame, width/2, 0, fromY, toY, limit=limit )
  xR = avoidGreen( frame, width/2, width, fromY, toY, limit=limit )
  return xL,xR


def processAvoidGreen( frame, debug=False ):
  height, width, colors = frame.shape
  offset = 0
  stripWidth = 70
  limit = 120
  topLR = stripLeftRight( frame, offset, offset+stripWidth, limit=limit )
  bottomLR = stripLeftRight( frame, height-offset-stripWidth, height-offset, limit=limit )
  if debug:
    cv2.imshow('image', frame)
    cv2.imwrite("green.jpg", frame)
  return topLR, bottomLR

if __name__ == "__main__":
  imgmain( sys.argv, processAvoidGreen ) #processGreen )
 
