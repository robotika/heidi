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
from ardrone2 import ARDrone2, ManualControlException, manualControl
import viewlog
from viewer import getCombinedPose # TODO refactoring

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
    hover = False
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
        # temporary workaround for log files without (frameNumber, timestamp)
        if len( drone.lastImageResult ) == 2 and len( drone.lastImageResult[0] ) == 2:
          (frameNumber, timestamp), lastRect = drone.lastImageResult
          viewlog.dumpCamera( "tmp_%04d.jpg" % frameNumber, 0 )
        else:
          lastRect = drone.lastImageResult
          frameNumber, timestamp = 0, 0
        if len(lastRect) > 0:
          angle = lastRect[0][2]
          videoTime = correctTimePeriod( timestamp/1000., ref=drone.time )
#          print angle, drone.time - videoTime
          rects = filterRectangles( lastRect )
          cp = classifyPath( [stripPose(r) for r in rects] )
          hover = False
          if cp != PATH_UNKNOWN:
            if pathType != PATH_UNKNOWN:
              if pathType != cp:
                print "TRANS", pathType, "->", cp
                if cp != PATH_CROSSING:
                  hover = True
            pathType = cp
          if pathType in [PATH_TURN_LEFT, PATH_TURN_RIGHT]:
            hover = True # it is simpler to turn angles with hover command

          if len(rects) > 0 and cp != PATH_CROSSING:
            pose = stripPose( rects[0] )
            print cp, frameNumber, "%.1f %d" % (pose[1], int(math.degrees(pose[2])))
            if pose[2] > math.radians(15): # angle
              sa = 0.1
            elif pose[2] < -math.radians(15):
              sa = -0.1
            else:
              sa = 0.0
            # compensate turns
#            if pathType == PATH_TURN_LEFT:
#              sa += 0.15
#            elif pathType == PATH_TURN_RIGHT:
#              sa -= 0.15

            if pose[1] > 0.1: # Y
              sy = 0.05
            elif pose[1] < -0.1:
              sy = -0.05
            else:
              sy = 0.0
            toDel = 0
            for oldTime, oldPose, oldAngles in poseHistory:
              toDel += 1
              if oldTime >= videoTime:
                break
#            print [int(math.degrees(x)) for x in oldAngles]
            poseHistory = poseHistory[:toDel] # keep history small

            for r in rects:
              coord = getCombinedPose( oldPose, stripPose( r ) )
              viewlog.dumpBeacon( (coord[0], coord[1]), index=3 )
              viewlog.dumpObstacles( [[(coord[0]-0.15*math.cos(coord[2]), coord[1]-0.15*math.sin(coord[2])), 
                                       (coord[0]+0.15*math.cos(coord[2]), coord[1]+0.15*math.sin(coord[2]))]] )
      if lastUpdate == None or drone.time > lastUpdate + 0.7:
        if hover:
          drone.update("AT*PCMD=%i,0,0,0,0,0\r") # drone.hover(0.1)
        else:
          drone.moveXYZA( sx, -sy/4., 0, 0 )
      else:
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

