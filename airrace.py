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
import math
import numpy as np
from pave import PaVE, isIFrame

g_index = 0

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
    # save image for simpler results review, angle is used as hash for search sub-sequence
    global g_index
    if len(result) > 0:
      angle = int(result[0][2])
    else:
      angle = 999
    cv2.imwrite( "tmp%03d_%d.jpg" % (g_index, angle), frame )
    g_index += 1
  return [((int(x),int(y)),(int(w),int(h)),int(a)) for ((x,y),(w,h),a) in result]

def filterRectangles( rects ):
  ret = []
  for (x,y),(w,h),a in rects:
    if w < h:
      w,h,a = h,w,a+90
    if w > 3*h:
      ret.append( ((x,y,),(w,h),a) )
  return ret

def stripPose( rect ):
  "return relative pose of image rectangle"
  (x,y),(w,h),a = rect
  assert w > 3*h, (w,h)  # 30cm long, 5cm wide ... i.e. should be 6 times
  a = -a-90
  if a > 90:
    a -= 180
  if a < -90:
    a += 180
  # TODO estimate z-coordinate and scale (x,y)
  # TODO image resolution as parameter? (possibility of direct or recorded video)
  return 720/2-y, x-1280/2, math.radians( a )

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

def testPaVEVideo( filename ):
  f = open( filename, "rb" )
  data = f.read(10000)
  pave = PaVE()
  cap = None
  total = 0
  while len(data) > 0:
    pave.append( data )
    header,payload = pave.extract()
    while payload:
      if isIFrame( header ):
        tmpFile = open( "tmp.bin", "wb" )
        tmpFile.write( payload )
        tmpFile.flush()
        tmpFile.close()
        cap = cv2.VideoCapture( "tmp.bin" )
        ret, frame = cap.read()
        assert ret
        if ret:
          print processFrame( frame, debug=True )
      header,payload = pave.extract()
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break
    data = f.read(10000)

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
      testPaVEVideo( filename )
    else:
      testVideo( filename )
  cv2.destroyAllWindows()

