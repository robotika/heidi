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

def processFrame( frame, debug=False ):
  gray = cv2.cvtColor( frame, cv2.COLOR_BGR2GRAY )
  ret, binary = cv2.threshold( gray, 0, 255, cv2.THRESH_OTSU )
  contours, hierarchy = cv2.findContours( binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE )
  if debug:
    cv2.drawContours(frame, contours, -1, (0,255,0), 3)
    cv2.imshow('image', frame)

def testFrame( filename ):
  img = cv2.imread( filename, 0 )
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

if __name__ == "__main__":
  if len(sys.argv) < 2:
    print __doc__
    sys.exit(2)
  testVideo( sys.argv[1] )
  cv2.destroyAllWindows()

