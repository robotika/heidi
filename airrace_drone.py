#!/usr/bin/python
"""
  AirRace competition in Vienna. See robotchallenge.org
  usage:
       ./airrace.py <TODO>
"""
import sys
import datetime
import multiprocessing
import cv2
import math
from pave import PaVE, isIFrame, frameNumber, timestamp, correctTimePeriod
from airrace import processFrame, allStripPoses
from sourcelogger import SourceLogger
from ardrone2 import ARDrone2, ManualControlException, manualControl, normalizeAnglePIPI, distance
import viewlog
from line import Line
from pose import Pose
from striploc import StripsLocalisation, REF_CIRCLE_RADIUS
from striploc import PATH_STRAIGHT, PATH_TURN_LEFT, PATH_TURN_RIGHT


MAX_ALLOWED_SPEED = 0.8
STRIPS_FAILURE_SPEED = 0.5 # ??? speed when localisation is not updated from last image ... maybe two images??
TRANSITION_SPEED = 0.4
NUM_FAST_STRIPS = 4 # now the same number for straight and turn segments -> TODO split
MAX_ALLOWED_VIDEO_DELAY = 2.0 # in seconds, then it will wait (desiredSpeed = 0.0)

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


class AirRaceDrone( ARDrone2 ):
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
    else:
      assert metaLog
      self.loggedVideoResult = SourceLogger( None, metaLog.getLog("cv2:") ).get
      self.startVideo( record=True, highResolution=self.videoHighResolution )

  def update( self, cmd="AT*COMWDG=%i,\r" ):
    ARDrone2.update( self, cmd )
    if self.loggedVideoResult != None:
      self.lastImageResult = self.loggedVideoResult()


def competeAirRace( drone, desiredHeight = 1.7 ):
  loops = []
  drone.speed = 0.1
  maxVideoDelay = 0.0
  maxControlGap = 0.0
  loc = StripsLocalisation()
  
  remainingFastStrips = 0
  desiredSpeed = TRANSITION_SPEED
  updateFailedCount = 0

  try:
    drone.wait(1.0)
    drone.setVideoChannel( front=False )    
    drone.takeoff()
    poseHistory = []
    startTime = drone.time
    while drone.time < startTime + 1.0:
      drone.update("AT*PCMD=%i,0,0,0,0,0\r") # drone.hover(1.0)
      poseHistory.append( (drone.time, (drone.coord[0], drone.coord[1], drone.heading), (drone.angleFB, drone.angleLR)) )
    magnetoOnStart = drone.magneto[:3]
    print "NAVI-ON"
    pathType = PATH_TURN_LEFT
    virtualRefCircle = None
    startTime = drone.time
    sx,sy,sz,sa = 0,0,0,0
    lastUpdate = None
    while drone.time < startTime + 600.0:
      sz = max( -0.2, min( 0.2, desiredHeight - drone.coord[2] ))
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
        loc.updateFrame( Pose( *oldPose ).add(tiltCompensation), allStripPoses( rects, highResolution=drone.videoHighResolution ) )
        if loc.pathType != pathType:
          print "TRANS", pathType, "->", loc.pathType
          if pathType == PATH_TURN_LEFT and loc.pathType == PATH_STRAIGHT:
            if len(loops) > 0:
              print "Loop %d, time %d" % (len(loops), drone.time-loops[-1])
            print "-----------------------------------------------"
            loops.append( drone.time )
          if drone.magneto[:3] == magnetoOnStart:
            print "!!!!!!!! COMPASS FAILURE !!!!!!!!"
          pathType = loc.pathType
          print "----"
          remainingFastStrips = NUM_FAST_STRIPS

        if loc.crossing:
          print "X", True, remainingFastStrips
        else:
          print pathType, loc.pathUpdated, remainingFastStrips
        if not loc.pathUpdated:
          updateFailedCount += 1
          if updateFailedCount > 1:
            print "UPDATE FAILED", updateFailedCount
        else:
          updateFailedCount = 0

        if remainingFastStrips > 0:
          remainingFastStrips -= 1
          desiredSpeed = MAX_ALLOWED_SPEED
          if not loc.pathUpdated and not loc.crossing:
            desiredSpeed = STRIPS_FAILURE_SPEED
        else:
          desiredSpeed = TRANSITION_SPEED
        if videoDelay > MAX_ALLOWED_VIDEO_DELAY:
          desiredSpeed = 0.0

        if drone.battery < 10:
          print "BATTERY LOW!", drone.battery

        for sp in allStripPoses( rects, highResolution=drone.videoHighResolution ):
          sPose = Pose( *oldPose ).add(tiltCompensation).add( sp )
          viewlog.dumpBeacon( sPose.coord(), index=3 )
          viewlog.dumpObstacles( [[(sPose.x-0.15*math.cos(sPose.heading), sPose.y-0.15*math.sin(sPose.heading)), 
                                       (sPose.x+0.15*math.cos(sPose.heading), sPose.y+0.15*math.sin(sPose.heading))]] )

      refCircle,refLine = loc.getRefCircleLine( Pose(drone.coord[0], drone.coord[1], drone.heading) )
      if refCircle == None and refLine == None and virtualRefCircle != None:
        refCircle = virtualRefCircle
      # error definition ... if you substract that you get desired position or angle
      # error is taken from the path point of view, x-path direction, y-positive left, angle-anticlockwise
      errY, errA = 0.0, 0.0
      assert refCircle == None or refLine == None # they cannot be both active at the same time
      if refCircle:
        if pathType == PATH_TURN_LEFT:
          errY = refCircle[1] - math.hypot( drone.coord[0]-refCircle[0][0], drone.coord[1]-refCircle[0][1] )
          errA = normalizeAnglePIPI( - math.atan2( refCircle[0][1] - drone.coord[1], refCircle[0][0] - drone.coord[0] ) 
                                      - math.radians(-90) + drone.heading )
        if pathType == PATH_TURN_RIGHT:
          errY = math.hypot( drone.coord[0]-refCircle[0][0], drone.coord[1]-refCircle[0][1] ) - refCircle[1]
          errA = normalizeAnglePIPI( - math.atan2( refCircle[0][1] - drone.coord[1], refCircle[0][0] - drone.coord[0] ) 
                                      - math.radians(90) + drone.heading )
      if refLine:
        errY = refLine.signedDistance( drone.coord )
        errA = normalizeAnglePIPI( drone.heading - refLine.angle )

      # get the height first
      if drone.coord[2] < desiredHeight - 0.1 and drone.time-startTime < 5.0:
        sx = 0.0
      if refCircle == None and refLine == None and virtualRefCircle == None:
        sx = 0.0 # wait for Z-up
        if drone.coord[2] > desiredHeight - 0.1:
          print "USING VIRTUAL LEFT TURN CIRCLE!"
          circCenter = Pose( drone.coord[0], drone.coord[1], drone.heading ).add( Pose(0.0, REF_CIRCLE_RADIUS, 0 )).coord()
          viewlog.dumpBeacon( circCenter, index=0 )
          virtualRefCircle = circCenter, REF_CIRCLE_RADIUS

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
  print "Loops", len(loops)-1, [int(now-prev) for prev,now in zip(loops[:-1],loops[1:])]
  print "Battery", drone.battery


if __name__ == "__main__": 
  import launcher
  launcher.launch( sys.argv, AirRaceDrone, competeAirRace )

