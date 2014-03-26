#!/usr/bin/python
"""
  AirRace competition in Vienna. See robotchallenge.org
  usage:
       ./airrace.py <image or video file> [<PaVE frame number>|<src_cv2_ refrence log file>]
"""
# for introduction of cv2 for Python have a look at
# http://docs.opencv.org/trunk/doc/py_tutorials/py_tutorials.html
import sys
import cv2
import math
import numpy as np
from pave import PaVE, isIFrame, frameNumber, timestamp
from pose import Pose

FRAMES_PER_INDEX = 15 # for simpler review of 15Hz images
THRESHOLD_FRACTION = 0.05

g_filename = None
g_mser = None

# Jakub's workaround for OpenCV 2.4.2 on linux
def arrayTo3d( contours ):
  contours3d = []
  for cnt in contours:
    cnt3d = np.zeros( (cnt.shape[0], 1, cnt.shape[1]), dtype=int )
    for ii in xrange(cnt.shape[0]):
      cnt3d[ii] = cnt[ii]
    contours3d.append(cnt3d)
  return contours3d




def processFrame( frame, debug=False ):
  result = []
  global g_mser
  global THRESHOLD_FRACTION
  if g_mser == None:
    g_mser = cv2.MSER( _delta = 10, _min_area=100, _max_area=300*50*2 )
  gray = cv2.cvtColor( frame, cv2.COLOR_BGR2GRAY )
  if g_mser:
    contours = g_mser.detect(gray, None)
    if cv2.__version__ == "2.4.2":
      contours = arrayTo3d( contours ) # Jakub's workaround for 2.4.2 on linux
  else:
    histogram = cv2.calcHist([gray],[0],None,[256],[0,256])
    s = 0
    for i, h in enumerate(histogram):
      s += h
      if s > THRESHOLD_FRACTION * 640 * 360:
        break
    ret, binary = cv2.threshold( gray, i, 255, cv2.THRESH_BINARY )
#    ret, binary = cv2.threshold( gray, 0, 255, cv2.THRESH_OTSU )
    contours, hierarchy = cv2.findContours( binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE )
  for cnt in contours:
    if g_mser == None:
      area = cv2.contourArea(cnt, oriented=True)
    if g_mser != None or (area > 100 and area < 100000):
      rect = cv2.minAreaRect(cnt)
      if g_mser == None or len(cnt)/float(rect[1][0]*rect[1][1]) > 0.70:
        result.append( rect )
  if g_mser != None:
    result = removeDuplicities( result )
  if debug:
    if g_mser == None:
      cv2.drawContours(frame, contours, -1, (0,255,0), 3)
    for rect in result:
      box = cv2.cv.BoxPoints(rect)
      box = np.int0(box)
      cv2.drawContours( frame,[box],0,(255,0,0),2)
  result = filterRectangles([((int(x),int(y)),(int(w),int(h)),int(a)) for ((x,y),(w,h),a) in result], minWidth=150/2)
  if debug:
    for rect in result:
      box = cv2.cv.BoxPoints(rect)
      box = np.int0(box)
      cv2.drawContours( frame,[box],0,(0,0,255),2)
    cv2.imshow('image', frame)
    # save image for simpler results review, angle is used as hash for search sub-sequence
    if g_filename:
      cv2.imwrite( g_filename, frame )
  return result

def filterRectangles( rects, minWidth=150 ):
  ret = []
  for (x,y),(w,h),a in rects:
    if w < h:
      w,h,a = h,w,a+90
    if w > 3*h and w >= minWidth:
      ret.append( ((x,y,),(w,h),a) )
  return ret

def stripPose( rect, highResolution=True ):
  "return relative pose of image rectangle"
  (x,y),(w,h),a = rect
  assert w > 3*h, (w,h)  # 30cm long, 5cm wide ... i.e. should be 6 times
  a = -a-90
  if a > 90:
    a -= 180
  if a < -90:
    a += 180
  if w >= 5*h: # it should be 6x, but width is more precise
    scale = 0.3/float(w)
  else:
    scale = 0.05/float(h)
  if highResolution:
    return Pose( scale*(720/2-y), scale*(1280/2-x), math.radians( a ) )
  else:
    return Pose( scale*(360/2-y), scale*(640/2-x), math.radians( a ) )


def removeDuplicities( rectangles, desiredRatio=6.0 ):
  "for MSER remove multiple detections of the same strip"
  radius = 30
  ret = []
  for (x,y),(w,h),a in rectangles:
    for (x2,y2),(w2,h2),a2 in ret:
      if abs(x-x2) < radius and abs(y-y2) < radius:
        ratio = max(h,w)/float(min(h,w))
        ratio2 = max(h2,w2)/float(min(h2,w2))
        if abs(ratio-desiredRatio) < abs(ratio2-desiredRatio):
          # use the bigger one
          ret.remove( ((x2,y2),(w2,h2),a2) )
          ret.append( ((x,y),(w,h),a) )
        break
    else:
      ret.append( ((x,y),(w,h),a) )
  return ret


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

def testPaVEVideo( filename, onlyFrameNumber=None, refLog=None ):
  f = open( filename, "rb" )
  data = f.read(10000)
  pave = PaVE()
  cap = None
  total = 0
  while len(data) > 0:
    pave.append( data )
    header,payload = pave.extract()
    while payload:
      if isIFrame( header ) and (onlyFrameNumber == None or onlyFrameNumber==frameNumber( header )/FRAMES_PER_INDEX):
        tmpFile = open( "tmp.bin", "wb" )
        tmpFile.write( payload )
        tmpFile.flush()
        tmpFile.close()
        cap = cv2.VideoCapture( "tmp.bin" )
        ret, frame = cap.read()
        assert ret
        if ret:
          global g_filename
          g_filename = "tmp_%04d.jpg" % (frameNumber( header )/FRAMES_PER_INDEX)
          result = processFrame( frame, debug=True )
          if refLog != None:
            print refLog.readline().strip()
            (oldFrameNumber, oldTimestamp), oldResult = eval(refLog.readline().strip())
            assert oldFrameNumber == frameNumber(header), (oldFrameNumber, frameNumber(header))
            assert oldTimestamp == timestamp(header), (oldTimestamp, timestamp(header))
            print ((frameNumber(header), timestamp(header)), result)
            # assert oldResult == result, oldResult # potential difference linux/windows
          else:
            print frameNumber( header )/FRAMES_PER_INDEX,  result
        if onlyFrameNumber:
          cv2.waitKey(0)
          return

      header,payload = pave.extract()
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break
    data = f.read(10000)
  if refLog != None: # complete termination line
    print refLog.readline().strip()


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
      if len(sys.argv) > 2:
        if "src_cv2_" in sys.argv[2]:
          testPaVEVideo( filename, refLog=open(sys.argv[2]) )
        else:
          testPaVEVideo( filename, onlyFrameNumber=int(eval(sys.argv[2])) )
      else:
        testPaVEVideo( filename )
    else:
      testVideo( filename )
  cv2.destroyAllWindows()

