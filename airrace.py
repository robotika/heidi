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
import numpy as np
from paveproxy import PaVEProxy

def processFrame( frame, debug=False ):
  result = []
  gray = cv2.cvtColor( frame, cv2.COLOR_BGR2GRAY )
  ret, binary = cv2.threshold( gray, 0, 255, cv2.THRESH_OTSU )
  contours, hierarchy = cv2.findContours( binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE )
  for cnt in contours:
    area = cv2.contourArea(cnt)
    if area > 100 and area < 100000:
      rect = cv2.minAreaRect(cnt)
      result.append( rect )
  if debug:
    cv2.drawContours(frame, contours, -1, (0,255,0), 3)
    for rect in result:
      box = cv2.cv.BoxPoints(rect)
      box = np.int0(box)
      cv2.drawContours( frame,[box],0,(0,0,255),2)
    cv2.imshow('image', frame)
  return result

def testFrame( filename ):
  img = cv2.imread( filename, cv2.CV_LOAD_IMAGE_COLOR )
  processFrame( img, debug=True )
  cv2.waitKey(0)

def testVideo( filename ):
  cap = cv2.VideoCapture( filename )
  while(cap.isOpened()):
    ret, frame = cap.read()
    if ret:
      processFrame( frame, debug=True )
    else:
      break
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break
  cap.release()


def isParrotVideo( filename ):
  return open( filename ).read(4) == "PaVE"


if __name__ == "__main__":
  if len(sys.argv) < 2:
    print __doc__
    sys.exit(2)
  filename = sys.argv[1]
  if filename.endswith(".jpg"):
    testFrame( filename )
  else:
    if isParrotVideo( filename ):
      pp = PaVEProxy( open( filename, "rb" ).read )
      testVideo( 'tcp://localhost:50007/' )
    else:
      testVideo( filename )
  cv2.destroyAllWindows()

