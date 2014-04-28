#!/usr/bin/python
"""
  Robotem Rovne competition in Pisek. See www.kufr.cz
  usage:
       ./rr_drone.py <TODO>
"""
# based on airrace_drone.py
import sys
import datetime
import multiprocessing
import cv2
import math
import numpy as np

from pave import PaVE, isIFrame, frameNumber, timestamp, correctTimePeriod
from sourcelogger import SourceLogger
from ardrone2 import ARDrone2, ManualControlException, manualControl, normalizeAnglePIPI, distance
import viewlog
from line import Line
from pose import Pose

from airrace import main as imgmain # image debugging TODO move to launcher

MAX_ALLOWED_SPEED = 0.8
MAX_ALLOWED_VIDEO_DELAY = 2.0 # in seconds, then it will wait (desiredSpeed = 0.0)

g_mser = None

def approx4pts( poly ):
  ret = cv2.approxPolyDP( poly, epsilon = 30, closed=True )
  if len(ret) == 4:
    return ret
  if len(ret) < 4:
    return cv2.approxPolyDP( poly, epsilon = 20, closed=True )
  return cv2.approxPolyDP( poly, epsilon = 50, closed=True )


def processFrame( frame, debug=False ):
  global g_mser
  midY = frame.shape[0]/2+100
  stripWidth = 120
  if g_mser == None:
    g_mser = cv2.MSER( _delta = 10, _min_area=100, _max_area=stripWidth*1000 )
  imgStrip = frame[ midY:midY+stripWidth, 0:frame.shape[1] ]
  b,g,r = cv2.split( imgStrip )
  gray = b
  contours = g_mser.detect(gray, None)
  result = []
  hulls = []
  for cnt in contours:
    (x1,y1),(x2,y2) = np.amin( cnt, axis=0 ), np.amax( cnt, axis=0 )
    if y1 == 0 and y2 == stripWidth-1: # i.e. whole strip
      if x1 > 0 and x2 < frame.shape[1]-1:
        hull = cv2.convexHull(cnt.reshape(-1, 1, 2))
        for h in hull:
          h[0][1] += midY
        hulls.append( hull )
        result.append( approx4pts( hull ) )
  if debug:
    cv2.polylines(frame, hulls, 2, (0, 255, 0), 2)
    for trapezoid in result:
      cv2.drawContours( frame,[np.int0(trapezoid)],0,(255,0,0),2)
    cv2.imshow('image', frame)
  return result

def timeName( prefix, ext ):
  dt = datetime.datetime.now()
  filename = prefix + dt.strftime("%y%m%d_%H%M%S.") + ext
  return filename

g_pave = None

def wrapper( packet ):
  global g_pave
  if g_pave == None:
    g_pave = PaVE()
  g_pave.append( packet )
  header,payload = g_pave.extract()
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
        return (frameNumber( header ), timestamp(header)), processFrame( frame, debug=False )
    header,payload = g_pave.extract()

g_queueResults = multiprocessing.Queue()

def getOrNone():
  if g_queueResults.empty():
    return None
  return g_queueResults.get()


class RobotemRovneDrone( ARDrone2 ):
  def __init__( self, replayLog=None, speed = 0.2, skipConfigure=False, metaLog=None, console=None ):
    self.loggedVideoResult = None
    self.lastImageResult = None
    self.videoHighResolution = True
    ARDrone2.__init__( self, replayLog, speed, skipConfigure, metaLog, console )
    if replayLog == None:
      name = timeName( "logs/src_cv2_", "log" ) 
      metaLog.write("cv2: "+name+'\n' )
      self.loggedVideoResult = SourceLogger( getOrNone, name ).get
      self.startVideo( wrapper, g_queueResults, record=True, highResolution=self.videoHighResolution )
    else:
      assert metaLog
      self.loggedVideoResult = SourceLogger( None, metaLog.getLog("cv2:") ).get
      self.startVideo( record=True, highResolution=self.videoHighResolution )

  def update( self, cmd="AT*COMWDG=%i,\r" ):
    ARDrone2.update( self, cmd )
    if self.loggedVideoResult != None:
      self.lastImageResult = self.loggedVideoResult()


def competeRobotemRovne( drone, desiredHeight = 1.5 ):
  drone.speed = 0.1
  maxVideoDelay = 0.0
  maxControlGap = 0.0
  desiredSpeed = MAX_ALLOWED_SPEED
  refLine = None
  
  try:
    drone.wait(1.0)
    drone.setVideoChannel( front=True )
    drone.takeoff()
    poseHistory = []
    startTime = drone.time
    while drone.time < startTime + 1.0:
      drone.update("AT*PCMD=%i,0,0,0,0,0\r") # drone.hover(1.0)
      poseHistory.append( (drone.time, (drone.coord[0], drone.coord[1], drone.heading), (drone.angleFB, drone.angleLR)) )
    magnetoOnStart = drone.magneto[:3]
    print "NAVI-ON"
    startTime = drone.time
    sx,sy,sz,sa = 0,0,0,0
    lastUpdate = None
    while drone.time < startTime + 600.0:

      altitude = desiredHeight
      if drone.altitudeData != None:
        altVision = drone.altitudeData[0]/1000.0
        altSonar = drone.altitudeData[3]/1000.0
        altitude = (altSonar+altVision)/2.0
        # TODO selection based on history? panic when min/max too far??
        if abs(altSonar-altVision) > 0.5:
          print altSonar, altVision
          altitude = max( altSonar, altVision ) # sonar is 0.0 sometimes (no ECHO)

      sz = max( -0.2, min( 0.2, desiredHeight - altitude ))

      sx = max( 0, min( drone.speed, desiredSpeed - drone.vx ))

      if drone.lastImageResult:
        lastUpdate = drone.time
        assert len( drone.lastImageResult ) == 2 and len( drone.lastImageResult[0] ) == 2, drone.lastImageResult
        (frameNumber, timestamp), rects = drone.lastImageResult
        viewlog.dumpCamera( "tmp_%04d.jpg" % (frameNumber/15,), 0 )

        # keep history small
        videoTime = correctTimePeriod( timestamp/1000., ref=drone.time )
        videoDelay = drone.time - videoTime
        if videoDelay > 1.0:
          print "!DANGER! - video delay", videoDelay
        maxVideoDelay = max( videoDelay, maxVideoDelay )
        toDel = 0
        for oldTime, oldPose, oldAngles in poseHistory:
          toDel += 1
          if oldTime >= videoTime:
            break
        poseHistory = poseHistory[:toDel]

        tiltCompensation = Pose(desiredHeight*oldAngles[0], desiredHeight*oldAngles[1], 0) # TODO real height?
        print "FRAME", frameNumber/15, "[%.1f %.1f]" % (math.degrees(oldAngles[0]), math.degrees(oldAngles[1])),
#        loc.updateFrame( Pose( *oldPose ).add(tiltCompensation), allStripPoses( rects, highResolution=drone.videoHighResolution ) )

        if drone.battery < 10:
          print "BATTERY LOW!", drone.battery

        # height debugging
        #print "HEIGHT\t%d\t%d\t%.2f\t%d\t%d\t%d\t%d\t%d\t%d" % tuple([max([0]+[w for ((x,y),(w,h),a) in rects])] + list(drone.altitudeData[:4]) + list(drone.pressure) )

      # error definition ... if you substract that you get desired position or angle
      # error is taken from the path point of view, x-path direction, y-positive left, angle-anticlockwise
      errY, errA = 0.0, 0.0
      if refLine:
        errY = refLine.signedDistance( drone.coord )
        errA = normalizeAnglePIPI( drone.heading - refLine.angle )

      # get the height first
      if drone.coord[2] < desiredHeight - 0.1 and drone.time-startTime < 5.0:
        sx = 0.0
      # error correction
      # the goal is to have errY and errA zero in 1 second -> errY defines desired speed at given distance from path
      sy = max( -0.2, min( 0.2, -errY-drone.vy ))/2.0
      
      # there is no drone.va (i.e. derivative of heading) available at the moment ... 
      sa = max( -0.1, min( 0.1, -errA/2.0 ))*1.35*(desiredSpeed/0.4) # originally set for 0.4=OK

#      print "%0.2f\t%d\t%0.2f\t%0.2f\t%0.2f" % (errY, int(math.degrees(errA)), drone.vy, sy, sa)
      prevTime = drone.time
      drone.moveXYZA( sx, sy, sz, sa )
      maxControlGap = max( drone.time - prevTime, maxControlGap )
      poseHistory.append( (drone.time, (drone.coord[0], drone.coord[1], drone.heading), (drone.angleFB, drone.angleLR)) )
    print "NAVI-OFF", drone.time - startTime
    drone.hover(0.5)
    drone.land()
    drone.setVideoChannel( front=True )    
  except ManualControlException, e:
    print "ManualControlException"
    if drone.ctrlState == 3: # CTRL_FLYING=3 ... i.e. stop the current motion
      drone.hover(0.1)
    drone.land()
  drone.wait(1.0)
  drone.stopVideo()
  print "MaxVideoDelay", maxVideoDelay
  print "MaxControlGap", maxControlGap
  print "Battery", drone.battery


if __name__ == "__main__":
  if len(sys.argv) > 2 and sys.argv[1] == "img":
    imgmain( sys.argv[1:], processFrame )
    sys.exit( 0 )
  import launcher
  launcher.launch( sys.argv, RobotemRovneDrone, competeRobotemRovne )

