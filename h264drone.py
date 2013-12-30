#!/usr/bin/python
"""
  Experiment with H264 codec parsing for AR Drone 2.0 autonomous navigation
  usage:
       ./h264drone.py <task|reply> [<reply log> [F]]
"""
from ardrone2 import ARDrone2, ManualControlException, manualControl
from sourcelogger import SourceLogger

import sys
import datetime
import struct
import os
import time
from threading import Thread,Event,Lock
import multiprocessing

import viewlog

# import from the h264-drone-vision repository (https://github.com/robotika/h264-drone-vision)
sys.path.append( ".."+os.sep+"h264-drone-vision") 
import h264
h264.verbose = False
from h264nav import quadrantMotion

MAX_COUNT = 1000
MIN_STEP = 100


def timeName( prefix, ext ):
  dt = datetime.datetime.now()
  filename = prefix + dt.strftime("%y%m%d_%H%M%S.") + ext
  return filename


class PacketProcessor( Thread ):
  def __init__( self ):
    Thread.__init__( self )
    self.setDaemon( True )
    self.lock = Lock()
    self.buf = ""
    self.readyForProcessing = ""
    self._lastResult = None
    self.timestamp = None
    self.frameNum = None
    self.shouldIRun = Event()
    self.shouldIRun.set()
    self.start()

  def process( self, packet ):
    if packet.startswith("PaVE"): # PaVE (Parrot Video Encapsulation)
      if len(self.buf) >= 28:
        version, codec, headerSize, payloadSize = struct.unpack_from("BBHI", self.buf, 4 )
        assert version == 3, version
        assert codec == 4, codec
        frameNum, timestamp = struct.unpack_from("II", self.buf, 20 )
#        assert len(self.buf) == headerSize + payloadSize, str( (len(self.buf), headerSize, payloadSize) )
        if len(self.buf) == headerSize + payloadSize:
          self.lock.acquire()
#          if len( self.readyForProcessing ) > 0:
#            print "skipping", len(self.readyForProcessing)
          self.readyForProcessing = self.buf[headerSize:]
          self.timestamp = timestamp
          self.frameNum = frameNum
          self.lock.release()
        else:
          # this looks like frequent case - PaVE is probably also in the middle of the packets
          print "BAD PACKET", (len(self.buf), headerSize, payloadSize)
      self.buf = ""
    self.buf += packet

  def run(self):
    while True: #self.shouldIRun.isSet():
      if len( self.readyForProcessing) > 0:
        self.lock.acquire()
        tmp = self.readyForProcessing
        timestamp = self.timestamp
        frameNum = self.frameNum
        self.readyForProcessing = ""
        self.lock.release()
        mv = h264.parseFrame( tmp )
        self.lock.acquire()
        self._lastResult = frameNum, timestamp, quadrantMotion( mv )
        self.lock.release()
        print len(mv), self._lastResult

  def lastResult(self):
    self.lock.acquire()
    ret = self._lastResult
    self.lock.release()
    return ret 

  def requestStop(self):
    self.shouldIRun.clear() 

g_pp = None

def wrapper( packet ):
#  print "Packet", len(packet)
  global g_pp
  if g_pp == None:
    g_pp = PacketProcessor()
  g_pp.process( packet )
  return g_pp.lastResult()


def dummyPacketProcessor( packet ):
  print len(packet)
  packetLog = open("packet.log", "a")
  packetLog.write( repr( packet ) + '\n' )
  packetLog.flush()

def replayPacketLog( filename, packetProcessor ):
  for line in open(filename):
    packet = eval(line)
    packetProcessor( packet )


# very very ugly :(
queueResults = multiprocessing.Queue()

def getOrNone():
  if queueResults.empty():
    return None
  return queueResults.get()

def h264drone( replayLog, metaLog, desiredSpeed = 1.0, timeout = 5.0 ):
  drone = ARDrone2( replayLog, metaLog=metaLog )
  if replayLog:
    for line in metaLog: # TODO refactoring
      print "XXLINE", line.strip()
      if line.startswith("h264:"):
        loggedResult = SourceLogger( None, line.split()[1].strip() ).get
        break
    drone.startVideo()
  else:
    name = timeName( "logs/src_h264_", "log" ) 
    metaLog.write("h264: "+name+'\n' )
    loggedResult = SourceLogger( getOrNone, name ).get
    drone.startVideo( wrapper, queueResults )

  if drone.userEmergencyLanding:
    drone.reset()
  try:
    drone.wait(1.0)
    drone.takeoff( enabledCorrections = False )
    # TODO some flying
    startTime = drone.time
    vz = 0.0
    while drone.time-startTime < timeout:
#      print "SPEED", drone.vx
      if drone.vx > desiredSpeed:
        drone.moveXYZA( 0.0, 0.0, 0.0, vz )
      else:
        drone.moveXYZA( drone.speed, 0.0, 0.0, vz )
      tmp = loggedResult()
      if tmp != None:
        print "QUEUE", drone.time, tmp
        frameNum, timestamp, (left, right, up, down ) = tmp
        vz = 0.0
        if left + right < MAX_COUNT: # limited max number of movements in the whole image
          if down > up + MIN_STEP:
            # move up
            vz = drone.speed
          if up > down + MIN_STEP:
            # move down
            vz = -drone.speed
          
    drone.land()
    drone.wait(1.0)
  except ManualControlException, e:
    print "ManualControlException"
    manualControl( drone )
    if drone.ctrlState == 3: # CTRL_FLYING=3 ... i.e. stop the current motion
      drone.hover(0.1)
    drone.land()
  drone.wait(1.0)
  drone.stopVideo()
  drone.halt()

if __name__ == "__main__":
  if len(sys.argv) < 2:
    print __doc__
    sys.exit(2)

  # TODO unified launcher, similar to Eduro
  if len(sys.argv) > 3 and sys.argv[3] == 'F':
    g_checkAssert = False
  replayLog = None
  metaLog = None
  if len(sys.argv) > 2:
    if "meta" in sys.argv[2]:
      metaLog = open(sys.argv[2])
      for line in metaLog:
        if line.startswith("navdata:"):
          replayLog = line.split()[1].strip()
          break
    else:
      replayLog=sys.argv[2]
    viewlog.viewLogFile = open( "view.log", "w" )
    viewlog.dumpSharpsGeometry( [(0.18, 0.0, 0.0)] ) # front sonar
  else: # create new metaLog
    metaLog = open( datetime.datetime.now().strftime("logs/meta_%y%m%d_%H%M%S.log"), "w" )
    metaLog.write( str(sys.argv) + "\n" )    
    metaLog.flush()
  if sys.argv[1] == "replay":
    for replayLog in sys.argv[2:]:
      drone = ARDrone2( replayLog, skipConfigure=True )
      try:
        while True:
          drone.update()
          if drone.altitudeData:
            print "%d\t%.3f\t" % (drone.ctrlState, drone.coord[2]) + "\t".join([str(x) for x in drone.altitudeData])
      except EOFError:
        pass
  else:
    h264drone( replayLog=replayLog, metaLog=metaLog )

