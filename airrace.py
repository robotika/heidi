#!/usr/bin/python
"""
  AirRace competition in Vienna. See robotchallenge.org
  usage:
       ./airrace.py <TODO>
"""
# for introduction of cv2 for Python have a look at
# http://docs.opencv.org/trunk/doc/py_tutorials/py_tutorials.html
import sys
import cv2

def processFrame( img, debug=False ):
  if debug:
    cv2.imshow('image',img)

def testFrame( filename ):
  img = cv2.imread( filename, 0 )
  processFrame( img, debug=True )
  cv2.waitKey(0)

if __name__ == "__main__":
  if len(sys.argv) < 2:
    print __doc__
    sys.exit(2)
  testFrame( sys.argv[1] )
  cv2.destroyAllWindows()

