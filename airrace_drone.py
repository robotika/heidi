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
from airrace import processFrame, filterRectangles, stripPose, classifyPath
from airrace import PATH_UNKNOWN, PATH_STRAIGHT, PATH_CROSSING, PATH_TURN_LEFT, PATH_TURN_RIGHT
from sourcelogger import SourceLogger
from ardrone2 import ARDrone2, ManualControlException, manualControl, normalizeAnglePIPI
import viewlog
from viewer import getCombinedPose # TODO refactoring

REF_CIRCLE_RADIUS = 1.4 # TODO measure in real arena!

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
    ARDrone2.__init__( self, replayLog, speed, skipConfigure, metaLog, console )
    if replayLog == None:
      name = timeName( "logs/src_cv2_", "log" ) 
      metaLog.write("cv2: "+name+'\n' )
      self.loggedVideoResult = SourceLogger( getOrNone, name ).get
      self.startVideo( wrapper, g_queueResults, record=True )
    else:
      assert metaLog
      for line in metaLog: # TODO refactoring
        print "XXLINE", line.strip()
        if line.startswith("cv2:"):
          self.loggedVideoResult = SourceLogger( None, line.split()[1].strip() ).get
          break
      self.startVideo( record=True )

  def update( self, cmd="AT*COMWDG=%i,\r" ):
    ARDrone2.update( self, cmd )
    if self.loggedVideoResult != None:
      self.lastImageResult = self.loggedVideoResult()


def competeAirRace( drone, desiredSpeed = 0.5, desiredHeight = 1.5 ):
  drone.speed = 0.1
  try:
    drone.wait(1.0)
    drone.setVideoChannel( front=False )    
    drone.takeoff()
    poseHistory = []
    startTime = drone.time
    while drone.time < startTime + 1.0:
      drone.update("AT*PCMD=%i,0,0,0,0,0\r") # drone.hover(1.0)
      poseHistory.append( (drone.time, (drone.coord[0], drone.coord[1], drone.heading), (drone.angleFB, drone.angleLR)) )
    print "NAVI-ON"
    pathType = PATH_TURN_LEFT
    refCircle = None
    refLine = None
    startTime = drone.time
    sx,sy,sz,sa = 0,0,0,0
    lastUpdate = None
    while drone.time < startTime + 120.0:
      # keep height approx at 1.5m
      if drone.coord[2] < desiredHeight-0.1:
        sz = 0.1
      elif drone.coord[2] > desiredHeight+0.1:
        sz = -0.1
      else:
        sz = 0
      # keep speed at max 1m/s
      if drone.vx > desiredSpeed:
        sx = 0
      else:
        sx = drone.speed

      if drone.lastImageResult:
        lastUpdate = drone.time
        assert len( drone.lastImageResult ) == 2 and len( drone.lastImageResult[0] ) == 2, drone.lastImageResult
        (frameNumber, timestamp), lastRect = drone.lastImageResult
        viewlog.dumpCamera( "tmp_%04d.jpg" % frameNumber, 0 )

        rects = filterRectangles( lastRect )
        cp = classifyPath( [stripPose(r) for r in rects] )
        if cp != PATH_UNKNOWN:
          if pathType != cp:
            print "TRANS", pathType, "->", cp
          pathType = cp
        print "FRAME", frameNumber, cp, pathType

        # keep history small
        videoTime = correctTimePeriod( timestamp/1000., ref=drone.time )
        toDel = 0
        for oldTime, oldPose, oldAngles in poseHistory:
          toDel += 1
          if oldTime >= videoTime:
            break
        poseHistory = poseHistory[:toDel]

        for r in rects:
          sPose = getCombinedPose( oldPose, stripPose( r ) )
          if pathType == PATH_TURN_LEFT:
            circPose = getCombinedPose( sPose, (0.0, REF_CIRCLE_RADIUS, 0 ))
            viewlog.dumpBeacon( (circPose[0], circPose[1]), index=0 )
            refCircle = (circPose[0], circPose[1]), REF_CIRCLE_RADIUS
          elif pathType == PATH_TURN_RIGHT:
            circPose = getCombinedPose( sPose, (0.0, -REF_CIRCLE_RADIUS, 0 ))
            viewlog.dumpBeacon( (circPose[0], circPose[1]), index=1 )
            refCircle = (circPose[0], circPose[1]), REF_CIRCLE_RADIUS
          else:
            refCircle = None
          viewlog.dumpBeacon( (sPose[0], sPose[1]), index=3 )
          viewlog.dumpObstacles( [[(sPose[0]-0.15*math.cos(sPose[2]), sPose[1]-0.15*math.sin(sPose[2])), 
                                       (sPose[0]+0.15*math.cos(sPose[2]), sPose[1]+0.15*math.sin(sPose[2]))]] )

      # error definition ... if you substract that you get desired position or angle
      # error is taken from the path point of view, x-path direction, y-positive left, angle-anticlockwise
      errY, errA = 0.0, 0.0
      if refCircle:
        if pathType == PATH_TURN_LEFT:
          errY = refCircle[1] - math.hypot( drone.coord[0]-refCircle[0][0], drone.coord[1]-refCircle[0][1] )
          errA = normalizeAnglePIPI( - math.atan2( refCircle[0][1] - drone.coord[1], refCircle[0][0] - drone.coord[0] ) 
                                      - math.radians(-90) + drone.heading )
        if pathType == PATH_TURN_RIGHT:
          errY = math.hypot( drone.coord[0]-refCircle[0][0], drone.coord[1]-refCircle[0][1] ) - refCircle[1]
          errA = normalizeAnglePIPI( math.atan2( refCircle[0][1] - drone.coord[1], refCircle[0][0] - drone.coord[0] ) 
                                      + math.radians(-90) - drone.heading )

      # now we test turns only -> land at the end of turn
      if pathType not in [PATH_TURN_LEFT, PATH_TURN_RIGHT]:
        break

      # error correction
      # the goal is to have errY and errA zero in 1 second -> errY defines desired speed at given distance from path
      sy = max( -0.2, min( 0.2, -errY-drone.vy ))/2.0
      
      # there is no drone.va (i.e. derivative of heading) available at the moment ... 
      sa = max( -0.1, min( 0.1, -errA/2.0 ))*1.2

#      print "%0.2f\t%d\t%0.2f\t%0.2f\t%0.2f" % (errY, int(math.degrees(errA)), drone.vy, sy, sa)
      drone.moveXYZA( sx, sy, sz, sa )
      poseHistory.append( (drone.time, (drone.coord[0], drone.coord[1], drone.heading), (drone.angleFB, drone.angleLR)) )
    print "NAVI-OFF"
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


if __name__ == "__main__": 
  import launcher
  launcher.launch( sys.argv, AirRaceDrone, competeAirRace )

