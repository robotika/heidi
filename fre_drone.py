#!/usr/bin/python
"""
  Field Robot Event 2014 (ver0 - autonomous landing on heliport sign)
  usage:
       ./fre_drone.py <task>
"""
import sys
import datetime
import multiprocessing
import cv2
import math
import numpy as np

from pave import PaVE, isIFrame, frameNumber, timestamp, correctTimePeriod, frameEncodedWidth, frameEncodedHeight

from sourcelogger import SourceLogger
from ardrone2 import ARDrone2, ManualControlException, manualControl, normalizeAnglePIPI, distance
import viewlog
from line import Line
from pose import Pose

from green import processAvoidGreen
import cvideo

#from rr_drone import downloadOldVideo
MAX_ALLOWED_SPEED = 0.8

###### -------------------- ready for major refactoring -------------------------

def timeName( prefix, ext ):
  dt = datetime.datetime.now()
  filename = prefix + dt.strftime("%y%m%d_%H%M%S.") + ext
  return filename

g_pave = None
g_img = None


def wrapper( packet ):
  global g_pave
  global g_img
  if g_pave == None:
    g_pave = PaVE()
    cvideo.init()
    g_img = np.zeros([360,640,3], dtype=np.uint8)
  g_pave.append( packet )
  header,payload = g_pave.extract()
  while payload:
    if isIFrame( header ):
      w,h = frameEncodedWidth(header), frameEncodedHeight(header)
      if g_img.shape[0] != h or g_img.shape[1] != w:
        print g_img.shape, (w,h)
        g_img = np.zeros([h,w,3], dtype=np.uint8)
      ret = cvideo.frame( g_img, isIFrame(header) and 1 or 0, payload )
      frame = g_img
      assert ret
      if ret:
        result = processAvoidGreen( frame, debug=False )
        return (frameNumber( header ), timestamp(header)), result
    header,payload = g_pave.extract()

g_queueResults = multiprocessing.Queue()

def getOrNone():
  if g_queueResults.empty():
    return None
  return g_queueResults.get()


###### --------------------------------------------------------------------------



# TODO for bottom cammera (!)
def project2plane( imgCoord, coord, height, heading, angleFB, angleLR ):
  FOW = math.radians(70)
  EPS = 0.0001
  x,y = imgCoord[0]-1280/2, 720/2-imgCoord[1]
  angleLR = -angleLR # we want to compensate the turn
  x,y = x*math.cos(angleLR)-y*math.sin(angleLR), y*math.cos(angleLR)+x*math.sin(angleLR)
  h = -x/1280*FOW + heading
  tilt = y/1280*FOW + angleFB
  if tilt > -EPS:
    return None # close to 0.0 AND projection behind drone
  dist = height/math.tan(-tilt)
  return (coord[0]+math.cos(h)*dist , coord[1]+math.sin(h)*dist)



class FieldRobotDrone( ARDrone2 ):
  def __init__( self, replayLog=None, speed = 0.2, skipConfigure=False, metaLog=None, console=None ):
    self.loggedVideoResult = None
    self.lastImageResult = None
    self.videoHighResolution = False
    ARDrone2.__init__( self, replayLog, speed, skipConfigure, metaLog, console )
    if replayLog == None:
      name = timeName( "logs/src_cv2_", "log" ) 
      metaLog.write("cv2: "+name+'\n' )
      self.loggedVideoResult = SourceLogger( getOrNone, name ).get
      self.startVideo( wrapper, g_queueResults, record=True, highResolution=self.videoHighResolution )
      self.startVideo( packetProcessor=None, inputQueue=None, record=True, highResolution=self.videoHighResolution )
    else:
      assert metaLog
      self.loggedVideoResult = SourceLogger( None, metaLog.getLog("cv2:") ).get
      self.startVideo( record=True, highResolution=self.videoHighResolution )

  def update( self, cmd="AT*COMWDG=%i,\r" ):
    ARDrone2.update( self, cmd )
    if self.loggedVideoResult != None:
      self.lastImageResult = self.loggedVideoResult()


def stayAtPosition( drone, desiredHeight = 1.5, timeout = 10.0 ):
  maxControlGap = 0.0
  desiredSpeed = MAX_ALLOWED_SPEED
  refPoint = (0,0)
  foundTagTime = None
  searchSeq = [(0,0), (2,0), (0,-2), (-2,0), (0,2), (0,0)]
  startTime = drone.time
  sx,sy,sz,sa = 0,0,0,0
  stepTime = drone.time
  while drone.time < startTime + timeout:
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
      if altitude > 2.5:
        # wind and "out of control"
        sz = max( -0.5, min( 0.5, desiredHeight - altitude ))

      sx = 0.0 #max( 0, min( drone.speed, desiredSpeed - drone.vx ))

#        tiltCompensation = Pose(desiredHeight*oldAngles[0], desiredHeight*oldAngles[1], 0) # TODO real height?
#        print "FRAME", frameNumber/15, "[%.1f %.1f]" % (math.degrees(oldAngles[0]), math.degrees(oldAngles[1])),

#        if drone.battery < 10:
#          print "BATTERY LOW!", drone.battery


      # error definition ... if you substract that you get desired position or angle
      # error is taken from the path point of view, x-path direction, y-positive left, angle-anticlockwise
      errX, errY, errA = 0.0, 0.0, 0.0
#      if refLine:
#        errY = refLine.signedDistance( drone.coord )
#        errA = normalizeAnglePIPI( drone.heading - refLine.angle )

      if len(drone.visionTag) > 0:
        SCALE = 0.17/(2*74)
        tagX, tagY, tagDist = drone.visionTag[0][0], drone.visionTag[0][1], drone.visionTag[0][4]/100.0
        tiltCompensation = Pose(tagDist*drone.angleFB, tagDist*drone.angleLR, 0)
        pose = Pose(drone.coord[0], drone.coord[1], drone.heading).add(tiltCompensation)
        offset = Pose(tagDist*(480-tagY)*SCALE, tagDist*(tagX-640)*SCALE, 0.0)
        pose = pose.add( offset )
        refPoint = (pose.x, pose.y)
        if foundTagTime == None:
          print drone.visionTag
          print "%.2f\t%.2f\t%.2f\t%.1f\t%.1f" % (drone.time, tagDist*(tagY-480)*SCALE, tagDist*(tagX-640)*SCALE, math.degrees(drone.angleFB), math.degrees(drone.angleLR))
          print refPoint
        foundTagTime = drone.time
      else:
        if foundTagTime != None and drone.time - foundTagTime > 3.0:
          foundTagTime = None
          print "LOST TAG"
          searchSeq = [(x+drone.coord[0], y+drone.coord[1]) for (x,y) in [(0,0), (2,0), (0,-2), (-2,0), (0,2), (0,0)]]

      if foundTagTime == None and len(searchSeq) > 0 and drone.time - stepTime > 3.0:
        refPoint = searchSeq[0]
        searchSeq = searchSeq[1:]
        print "STEP", refPoint
        stepTime = drone.time

      if refPoint:
        pose = Pose(drone.coord[0], drone.coord[1], drone.heading)
        landPose = Pose( refPoint[0], refPoint[1], drone.heading )  # ignore heading for the moment
        diff = pose.sub( landPose )
        #print diff
        errX, errY = diff.x, diff.y



      # get the height first
#      if drone.coord[2] < desiredHeight - 0.1 and drone.time-startTime < 5.0:
#        sx = 0.0
      # error correction
      # the goal is to have errY and errA zero in 1 second -> errY defines desired speed at given distance from path
      sx = max( -0.2, min( 0.2, -errX-drone.vx ))/2.0
      sy = max( -0.2, min( 0.2, -errY-drone.vy ))/2.0
      
      # there is no drone.va (i.e. derivative of heading) available at the moment ... 
      sa = max( -0.1, min( 0.1, -errA/2.0 ))*1.35*(desiredSpeed/0.4) # originally set for 0.4=OK

#      print "%0.2f\t%d\t%0.2f\t%0.2f\t%0.2f" % (errY, int(math.degrees(errA)), drone.vy, sy, sa)
      prevTime = drone.time
      drone.moveXYZA( sx, sy, sz, sa )
      maxControlGap = max( drone.time - prevTime, maxControlGap )
  return maxControlGap

def groundPose( (x,y), (xBottom,yBottom), scale ):
  a = math.atan2( xBottom-x, yBottom-y )
  return Pose( scale*(360/2-y), scale*(640/2-x), a )
  

def evalRowData( rowTopBottom ):
  (xL,xR),(xbL,xbR) = rowTopBottom
  if min(xL,xbL) > 0 and max(xR,xbR) < 640:
    widthTop, widthBottom = xR-xL, xbR-xbL
    if min(widthTop, widthBottom) > 150 and max(widthTop, widthBottom) < 350:
      return (xL+xR)/2, (xbL+xbR)/2

def followRow( drone, desiredHeight = 1.0, timeout = 10.0 ): 
  maxControlGap = 0.0
  maxVideoDelay = 0.0
  desiredSpeed = MAX_ALLOWED_SPEED
  startTime = drone.time
  sx,sy,sz,sa = 0,0,0,0
  lastUpdate = None
  refLine = None
  while drone.time < startTime + timeout:
    altitude = desiredHeight
    if drone.altitudeData != None:
      altVision = drone.altitudeData[0]/1000.0
      altSonar = drone.altitudeData[3]/1000.0
      altitude = (altSonar+altVision)/2.0
      # TODO selection based on history? panic when min/max too far??
      if abs(altSonar-altVision) > 0.5:
#        print altSonar, altVision
        altitude = max( altSonar, altVision ) # sonar is 0.0 sometimes (no ECHO)
    sz = max( -0.2, min( 0.2, desiredHeight - altitude ))
    if altitude > 2.5:
      # wind and "out of control"
      sz = max( -0.5, min( 0.5, desiredHeight - altitude ))

    if drone.lastImageResult:
      lastUpdate = drone.time
      assert len( drone.lastImageResult ) == 2 and len( drone.lastImageResult[0] ) == 2, drone.lastImageResult
      (frameNumber, timestamp), rowTopBottom = drone.lastImageResult
      viewlog.dumpVideoFrame( frameNumber, timestamp )
      # keep history small
      videoTime = correctTimePeriod( timestamp/1000., ref=drone.time )
      videoDelay = drone.time - videoTime
      if videoDelay > 1.0:
        print "!DANGER! - video delay", videoDelay
      maxVideoDelay = max( videoDelay, maxVideoDelay )
      toDel = 0
      for oldTime, oldPose, oldAngles in drone.poseHistory:
        toDel += 1
        if oldTime >= videoTime:
          break
      drone.poseHistory = drone.poseHistory[:toDel]
      tiltCompensation = Pose(desiredHeight*oldAngles[0], desiredHeight*oldAngles[1], 0) # TODO real height?      
      validRow = evalRowData( rowTopBottom )
      print "FRAME", frameNumber/15, "[%.1f %.1f]" % (math.degrees(oldAngles[0]), math.degrees(oldAngles[1])), validRow
      if validRow:
        sp = groundPose( *rowTopBottom, scale=0.75/((validRow[0]+validRow[1])/2.0))
        sPose = Pose( *oldPose ).add(tiltCompensation).add( sp )
        refLine = Line( (sPose.x,sPose.y), (sPose.x + math.cos(sPose.heading), sPose.y + math.sin(sPose.heading)) )

    errY, errA = 0.0, 0.0
    if refLine:
      errY = refLine.signedDistance( drone.coord )
      errA = normalizeAnglePIPI( drone.heading - refLine.angle )


    sx = max( 0, min( drone.speed, desiredSpeed - drone.vx ))
    sy = max( -0.2, min( 0.2, -errY-drone.vy ))/2.0
    sa = max( -0.1, min( 0.1, -errA/2.0 ))*1.35*(desiredSpeed/0.4)
    prevTime = drone.time
    drone.moveXYZA( sx, sy, sz, sa )
    drone.poseHistory.append( (drone.time, (drone.coord[0], drone.coord[1], drone.heading), (drone.angleFB, drone.angleLR)) )
    maxControlGap = max( drone.time - prevTime, maxControlGap )
  return maxControlGap


def competeFieldRobot( drone, desiredHeight = 1.5 ):
  drone.speed = 0.1
  maxVideoDelay = 0.0
  maxControlGap = 0.0
  
  try:
    drone.wait(1.0)
    drone.setVideoChannel( front=False )
    drone.takeoff()
    drone.poseHistory = []
 
    print "NAVI-ON"
    startTime = drone.time
#    maxControlGap = stayAtPosition( drone, desiredHeight=desiredHeight, timeout=30.0 )
    maxControlGap = followRow( drone, desiredHeight=desiredHeight, timeout=20.0 )
    lastUpdate = None
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
#  print "MaxVideoDelay", maxVideoDelay
  print "MaxControlGap", maxControlGap
  print "Battery", drone.battery


if __name__ == "__main__":
  if len(sys.argv) > 2 and sys.argv[1] == "img":
    imgmain( sys.argv[1:], processFrame )
    sys.exit( 0 )
  import launcher
  launcher.launch( sys.argv, FieldRobotDrone, competeFieldRobot )

