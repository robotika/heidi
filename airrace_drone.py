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
from pave import PaVE, isIFrame
from airrace import processFrame
from sourcelogger import SourceLogger
from ardrone2 import ARDrone2, ManualControlException, manualControl


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
        return processFrame( frame, debug=False )
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

  def update( self, cmd="AT*COMWDG=%i,\r" ):
    ARDrone2.update( self, cmd )
    if self.loggedVideoResult != None:
      self.lastImageResult = self.loggedVideoResult()


def competeAirRace( drone ):
  try:
    drone.wait(1.0)
    drone.setVideoChannel( front=False )    
    drone.takeoff()
    drone.hover(1.0)
    drone.land()
    drone.setVideoChannel( front=True )    
  except ManualControlException, e:
    print "ManualControlException"
    drone.land()
  drone.wait(1.0)
  drone.stopVideo()


if __name__ == "__main__": 
  import launcher
  launcher.launch( sys.argv, AirRaceDrone, competeAirRace )

