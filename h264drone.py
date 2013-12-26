#!/usr/bin/python
"""
  Experiment with H264 codec parsing for AR Drone 2.0 autonomous navigation
  usage:
       ./h264drone.py <task|reply> [<reply log> [F]]
"""
from ardrone2 import ARDrone2, ManualControlException

import sys
import datetime
import struct


class PacketProcessor:
  def __init__( self ):
    self.buf = ""
  def process( self, packet ):
    if packet.startswith("PaVE"): # PaVE (Parrot Video Encapsulation)
      if len(self.buf) >= 12:
        version, codec, headerSize, payloadSize = struct.unpack_from("BBHI", self.buf, 4 )
        assert version == 3, version
        assert codec == 4, codec
        assert len(self.buf) == headerSize + payloadSize, str(len(self.buf), headerSize, payloadSize)       
      self.buf = ""
    self.buf += packet


def dummyPacketProcessor( packet ):
  print len(packet)
  packetLog = open("packet.log", "a")
  packetLog.write( repr( packet ) + '\n' )
  packetLog.flush()

def replayPacketLog( filename, packetProcessor ):
  for line in open(filename):
    packet = eval(line)
    packetProcessor( packet )


def h264drone( replayLog, metaLog=None ):
  drone = ARDrone2( replayLog, metaLog=metaLog )
  drone.startVideo( dummyPacketProcessor )
  if drone.userEmergencyLanding:
    drone.reset()
  try:
    drone.wait(1.0)
    #drone.takeoff( enabledCorrections = False )
    # TODO some flying
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
#  pp = PacketProcessor()
#  replayPacketLog( "packet1.log", pp.process )
#  sys.exit(0)
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

