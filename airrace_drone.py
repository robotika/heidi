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
from pave import PaVE, isIFrame, frameNumber, timestamp
from airrace import processFrame, filterRectangles, stripPose
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


def competeAirRace( drone, desiredSpeed = 1.5, desiredHeight = 1.5 ):
  drone.speed = 0.05
  try:
    drone.wait(1.0)
    drone.setVideoChannel( front=False )    
    drone.takeoff()
    drone.hover(1.0)
    print "NAVI-ON"
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
          indexStamp, lastRect = drone.lastImageResult
        else:
          lastRect = drone.lastImageResult
        if len(lastRect) > 0:
          angle = lastRect[0][2]
          print angle
          rects = filterRectangles( lastRect )
          if len(rects) > 0:
            pose = stripPose( rects[0] )
            print pose, "(%.2f %.2f %.2f)" % drone.coord, " heading=%.1f" % math.degrees(drone.heading)
            if pose[2] > math.radians(15): # angle
              sa = 0.1
            elif pose[2] < -math.radians(15):
              sa = -0.1
            else:
              sa = 0.0
            if pose[1] > 0.1: # Y
              sy = 0.05
            elif pose[1] < -0.1:
              sy = -0.05
            else:
              sy = 0.0
            for r in rects:
              coord = getCombinedPose( (drone.coord[0], drone.coord[1], drone.heading), stripPose( r ) )
              viewlog.dumpBeacon( (coord[0], coord[1]), index=3 )
      if lastUpdate == None or drone.time > lastUpdate + 0.7:
        drone.update("AT*PCMD=%i,0,0,0,0,0\r") # drone.hover(0.1)
      else:
        drone.moveXYZA( sx, sy, sz, sa )
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

