#!/usr/bin/python
"""
  AR Drone 2.0 Python interface
  usage:
       ./ardrone2.py <task|replay> [<replay log> [F]]
"""
# interface based on https://github.com/venthur/python-ardrone
import sys
import datetime
import struct
import socket
import gzip
import math
import os

if sys.platform == 'win32':
  windows = True
else:
  windows = False

if windows:
  import msvcrt  # for kbhit
else:
  import pygame

import viewlog
from line import Line

from navdata import parseTimeTag, parseDemoTag, parseVisionDetectTag, parseRawMeasuresTag, parseAltitudeTag, parseMagnetoTag, parsePressureRawTag, parseGPSTag

from multiprocessing import Process, Queue
from video import logVideoStream

from sonar import Sonar
from sourcelogger import SourceLogger

from ro import loadWaypoints, waypoints2dirDist

HOST = '192.168.1.1'    # The remote host
VIDEO_RECORDER_PORT = 5553
NAVDATA_PORT = 5554
VIDEO_PORT = 5555
COMMAND_PORT = 5556
CONTROL_PORT = 5559

SOCKET_TIMEOUT = 0.1

# TODO refactoring - why is it also in navdata
NAVDATA_DEMO_TAG = 0
NAVDATA_TIME_TAG = 1
NAVDATA_RAW_MEASURES_TAG = 2
NAVDATA_ALTITUDE_TAG = 10
NAVDATA_VISION_DETECT_TAG = 16
NAVDATA_IPHONE_ANGLES_TAG = 18
NAVDATA_PRESSURE_RAW_TAG = 21
NAVDATA_MAGNETO_TAG = 22
NAVDATA_GPS_TAG = 27
NAVDATA_CKS_TAG = 0xFFFF


COLLISION_LIMIT_ACC = 400
COLLISION_LIMIT_ANGLE = math.radians(10.0)

# global var
global g_checkAssert
g_checkAssert = True # use command 'F' to disable it
global g_relogCmd
g_relogCmd = False # use command 'FL' to re-log commands (for next step refactoring)

def normalizeAnglePIPI( angle ):
  while angle < -math.pi:
    angle += 2*math.pi
  while angle > math.pi:
    angle -= 2*math.pi
  return angle 

def relCoord( x, y, heading ):
  "return coordinates of (0,0) in relative reference frame"
  c, s = math.cos(heading), math.sin(heading)
  return (-x*c -y*s,-y*c +x*s)

def myKbhit():
  if not msvcrt.kbhit():
    return 0
  key = msvcrt.getch()
  if key == '\xe0': # Arrows
    key = (key, msvcrt.getch())
  return key

def myPygame():
  events = pygame.event.get()
  for event in events:
    if event.type == pygame.KEYDOWN:
      return 1
  return 0

class MySockets:
  def __init__( self, logFilename, metaLog=None ):
    if logFilename.endswith(".gz"):
      self.logf = gzip.open( logFilename,"wb" )
    else:
      self.logf = open( logFilename,"wb" )
    cmdFilename = datetime.datetime.now().strftime("logs/atcmd_%y%m%d_%H%M%S.txt")
    if metaLog:
      metaLog.write("navdata: " + logFilename + "\n") # well, not the best place, but ...
      metaLog.write("atcmd: " + cmdFilename + "\n")
      metaLog.flush()
    self.logCmd = open( cmdFilename, "wb" )
    self.navdata = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.navdata.bind( ('',NAVDATA_PORT) )
    self.navdata.settimeout( SOCKET_TIMEOUT )
    self.navdata.sendto("\x01\x00\x00\x00", (HOST, NAVDATA_PORT)) 
    self.command = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  def update( self, cmd ):
    "send command and return navdata"
    data = None
    while data == None or len(data) == 0:
      try:
        data = self.navdata.recv(4094)
      except socket.timeout:
        print "Navdata TIMEOUT!"
        try:
          self.navdata.sendto("\x01\x00\x00\x00", (HOST, NAVDATA_PORT)) 
          data = self.navdata.recv(12048)
        except socket.timeout:
          print "Navdata FAILED 2nd test TIMEOUT!"
    self.logf.write(data)
    self.logf.flush()
    self.logCmd.write(cmd)
    self.logCmd.flush()
    self.command.sendto(cmd, (HOST, COMMAND_PORT))
    return data


class MyLogs:
  def __init__( self, filename, verbose=False, metaLog=None ):
    if filename.endswith(".gz"):
      self.f = gzip.open(filename, "rb")
    else:
      self.f = open(filename, "rb")
    self.verbose = verbose
    self.atcmd = None # default without checking    
    print "METALOG", metaLog
    if metaLog:
      self.atcmd = open( metaLog.getLog("atcmd:"), "rb" )
    self.relogCmd = None
    if g_relogCmd:
      self.relogCmd = open( "atcmd.txt", "wb" )

  def update( self, cmd ):
    "send command and return navdata"
    # ignore cmd for the moment
    if self.verbose:
      print cmd
    if self.atcmd:
      # verify AT command
      fileCmd = self.atcmd.read(len(cmd))
      if self.relogCmd:
        self.relogCmd.write(cmd)
        self.relogCmd.flush()
      if g_checkAssert:
        assert fileCmd==cmd, "Diff from file: %s, cmd %s" % (fileCmd, cmd)
    data = self.f.read(16) # header
    if len(data) == 0:
      raise EOFError
    tag = None
    while tag != 0xFFFF:
      option = self.f.read(4)
      tag, size = struct.unpack_from("HH", option)
      data += option + self.f.read(size-4)
    return data


class ManualControlException( Exception ):
  pass

def f2i(f):
    """Interpret IEEE-754 floating-point value as signed integer.

    Arguments:
    f -- floating point value
    """
    return struct.unpack('i', struct.pack('f', f))[0] 

def distance( planar1, planar2 ):
  "distane two planar points"
  x = planar1[0] - planar2[0]
  y = planar1[1] - planar2[1]
  return math.hypot(x, y)

class ARDrone2:
  def __init__( self, replayLog=None, speed = 0.2, skipConfigure=False, metaLog=None, console=None ):
    self.replayLog = replayLog
    self.metaLog = metaLog
    self.sonar = None
    self.sonarUpdate = None
    self.sonarData = None
    self.sonarIndex = None # index of current measurement
    self.console = None
    if replayLog:
      self.filename = replayLog
      self.io = MyLogs( self.filename, metaLog=metaLog )
      if self.metaLog:
#        for line in metaLog:
#          print "LINE", line.strip()
#          if line.startswith("sonar:"):
#            self.sonarUpdate = SourceLogger( None, line.split()[1].strip() ).get
#            break
        self.console = SourceLogger( None, metaLog.getLog("console:") ).get
    else:
      if not os.path.isdir("logs"):
        os.mkdir("logs")
      self.filename = datetime.datetime.now().strftime("logs/navdata_%y%m%d_%H%M%S.log.gz")
      self.io = MySockets( self.filename, metaLog=metaLog )
      if console == None:
        if windows:
          console = myKbhit
        else:
          pygame.init()
          screen = pygame.display.set_mode((100,100))
          console = myPygame
      if self.metaLog:
#        sonarFilename = datetime.datetime.now().strftime("logs/sonar_src_%y%m%d_%H%M%S.log")
#        self.metaLog.write("sonar: "+sonarFilename+'\n' )
#        self.sonar = Sonar()
#        self.sonarUpdate = SourceLogger( self.sonar.data, sonarFilename ).get
#        self.sonar.start()
        consoleFilename = datetime.datetime.now().strftime("logs/console_src_%y%m%d_%H%M%S.log")
        self.metaLog.write("console: "+consoleFilename+'\n' )
        self.console = SourceLogger( console, consoleFilename ).get
      else:
        self.console = SourceLogger( console, 'metalog' ).get
    print self.filename
    self.speed = speed
    self.manualControl = False
    self.lastSeq = 0
    self.time = None
    self.ctrlAck = False # ACK_BIT is the 6th bit of ardrone_state
    self.ctrlState = None
    self.userEmergencyLanding = None
    self.battery = None
    self.acc = None # info about raw accelerometer readings
    self.pressure = None # raw data
    self.collision = False

    self.coord = (0,0,0)
    self.heading = 0.0
    self.headingOffset = None
    self.angleLR = 0.0
    self.angleFB = 0.0
    self.dt = 0 # time derivative
    self.vx, self.vy = None, None # speed in XY-axis (expect refactoring)
    self.visionTag = []
    self.altitudeData = None
    self.compass = None # single heading
    self.magneto = None # whole array
    self.gps = None  # whole raw data (for now)
    if not skipConfigure:
      self.configure()
    for i in xrange(20):
      self.update()
    assert( self.time != None )
    print "Battery", self.battery
    self.videoQueue1 = None
    self.videoProcess1 = None
    self.videoQueue2 = None
    self.videoProcess2 = None

  def update( self, cmd="AT*COMWDG=%i,\r" ):
    "take care of self.seq"
    if self.sonarUpdate:
      tmp = self.sonarUpdate()
      if tmp != None and len(tmp) >= 2:
        self.sonarIndex = tmp[0]
        if tmp[1] != 0:
          self.sonarData = tmp[1]/100.0 # conversion to meters
          viewlog.dumpSharps( (self.coord[0], self.coord[1], self.heading), [self.sonarData] )
        else:
          self.sonarData = 100.0 # far ...
#      print self.sonarData

    if not self.manualControl and self.console != None:
      self.manualControl = self.console()
      if self.manualControl:
        raise ManualControlException()
    self.lastSeq +=1
    data = self.io.update( cmd % self.lastSeq )
    self.parseNavData( data )
    return data # is it still necessary??

  def confirmedConfig( self, cmd ):
    "wait for configuration confirmation"
    assert( not self.ctrlAck ) # should be cleared on start and after each configuration
    self.update( "AT*CONFIG_IDS=%i,\"0a1b2c3d\",\"0a1b2c3d\",\"0a1b2c3d\"\r" )
    self.update( cmd )
    for i in xrange(200): # timeout by time??
      self.update()
      if self.ctrlAck:
        break
    assert( self.ctrlAck )
    self.update("AT*CTRL=%i,5,0\r") # ACK_CONTROL_MODE=5
    for i in xrange(100): # timeout by time??
      self.update()
      if not self.ctrlAck:
        print cmd
        break
    assert( not self.ctrlAck )


  def configure( self ):
    "initial configuration"
    self.confirmedConfig( "AT*CONFIG=%i,\"custom:application_id\",\"0a1b2c3d\"\r" ) # one day use some defines ...
    self.confirmedConfig( "AT*CONFIG=%i,\"custom:profile_id\",\"0a1b2c3d\"\r" )
    self.confirmedConfig( "AT*CONFIG=%i,\"custom:session_id\",\"0a1b2c3d\"\r" )
  
    self.confirmedConfig( "AT*CONFIG=%i,\"general:navdata_demo\",\"FALSE\"\r" )
    self.confirmedConfig( "AT*CONFIG=%i,\"control:outdoor\",\"TRUE\"\r" )
    self.confirmedConfig( "AT*CONFIG=%i,\"control:flight_without_shell\",\"FALSE\"\r" )
    self.confirmedConfig( "AT*CONFIG=%i,\"control:indoor_control_vz_max\",\"2000\"\r" )
    self.confirmedConfig( "AT*CONFIG=%i,\"control:altitude_max\",\"5000\"\r" )
    # CAD_TYPE_MULTIPLE_DETECTION_MODE = 10, CAD_TYPE_ORIENTED_COCARDE_BW=12, CAD_TYPE_VISION_V2 = 13
    self.confirmedConfig( "AT*CONFIG=%i,\"detect:detect_type\",\"12\"\r") 
    # TAG_TYPE_SHELL_TAG=1, TAG_TYPE_ROUNDEL=2, TAG_TYPE_ORIENTED_ROUNDEL=4, TAG_TYPE_STRIPE=8
    # TAG_TYPE_SHELL_TAG_V2=32, TAG_TYPE_BLACK_ROUNDEL=128
    self.confirmedConfig( "AT*CONFIG=%i,\"detect:detections_select_h\",\"0\"\r" )
    self.confirmedConfig( "AT*CONFIG=%i,\"detect:detections_select_v_hsync\",\"0\"\r" )
    self.confirmedConfig( "AT*CONFIG=%i,\"detect:detections_select_v\",\"128\"\r" )
    self.confirmedConfig( "AT*CONFIG=%i,\"detect:enemy_colors\",\"2\"\r" ) # ARDRONE_DETECTION_COLOR_ORANGE_YELLOW=2
#    self.confirmedConfig( "AT*CONFIG=%i,\"detect:detect_type\",\"13\"\r") 
#    self.confirmedConfig( "AT*CONFIG=%i,\"detect:detections_select_h\",\"32\"\r" )
#    self.confirmedConfig( "AT*CONFIG=%i,\"detect:detections_select_v_hsync\",\"0\"\r" )
#    self.confirmedConfig( "AT*CONFIG=%i,\"detect:detections_select_v\",\"0\"\r" )
#    self.confirmedConfig( "AT*CONFIG=%i,\"detect:enemy_colors\",\"2\"\r" ) # ARDRONE_DETECTION_COLOR_ORANGE_YELLOW=2
#    self.confirmedConfig( "AT*CONFIG=%i,\"detect:enemy_without_shell\",\"0\"\r" )
    self.confirmedConfig( "AT*CONFIG=%i,\"video:video_channel\",\"0\"\r" ) # 0=VERTical, 1=HORIzontal view

  def setVideoChannel( self, front=True ):
    if front:
      self.confirmedConfig( "AT*CONFIG=%i,\"video:video_channel\",\"0\"\r" ) # 0=VERTical view
    else:
      self.confirmedConfig( "AT*CONFIG=%i,\"video:video_channel\",\"1\"\r" ) # 1=HORIzontal view

  def startVideo( self, packetProcessor=None, inputQueue=None, record=True, highResolution=True ):
    if self.videoQueue1 == None and self.videoProcess1 == None:
      if record:
        if highResolution:
          self.confirmedConfig( "AT*CONFIG=%i,\"video:video_codec\",\"130\"\r" ) # record MP4_360P_H264_720P_CODEC = 0x82,
        else:
          self.confirmedConfig( "AT*CONFIG=%i,\"video:video_codec\",\"136\"\r" ) # MP4_360P_H264_360P_CODEC = 0x88,
      else:
        if highResolution:
          self.confirmedConfig( "AT*CONFIG=%i,\"video:video_codec\",\"131\"\r" ) # H264_720P_CODEC = 0x83, 
        else:
          self.confirmedConfig( "AT*CONFIG=%i,\"video:video_codec\",\"129\"\r" ) # H264_360P_CODEC = 0x81,

      if self.replayLog == None:
        self.videoQueue1 = Queue()
        filename = datetime.datetime.now().strftime("logs/video_%y%m%d_%H%M%S.bin")
        if self.metaLog:
          self.metaLog.write("video: " + filename + "\n")
          self.metaLog.flush()
        if record:
          self.videoProcess1 = Process(target=logVideoStream, args=((HOST, VIDEO_PORT), filename, self.videoQueue1,))
        else:
          self.videoProcess1 = Process(target=logVideoStream, args=((HOST, VIDEO_PORT), filename, self.videoQueue1, \
            packetProcessor, inputQueue, False))
        self.videoProcess1.daemon = True
        self.videoProcess1.start()

        if record:
          self.videoQueue2 = Queue()
          filename = datetime.datetime.now().strftime("logs/video_rec_%y%m%d_%H%M%S.bin")
          if self.metaLog:
            self.metaLog.write("video_rec: " + filename + "\n")
            self.metaLog.flush()
          self.videoProcess2 = Process(target=logVideoStream, args=((HOST, VIDEO_RECORDER_PORT), filename, self.videoQueue2, \
              packetProcessor, inputQueue, True))
          self.videoProcess2.daemon = True
          self.videoProcess2.start()
      else:
        # replaying log
        if self.metaLog and record:
          viewlog.dumpVideo( self.metaLog.getLog("video_rec:") )

  def stopVideo( self ):
    if self.videoQueue1 != None and self.videoProcess1 != None:
      # TODO change configuration??
      # TODO mega assert to all video channels?
      print "sending video END ..."
      if self.replayLog == None:
        self.videoQueue1.put("The END")
        if self.videoQueue2:
          self.videoQueue2.put("The END")
        print "waiting for termination ..."
        for i in xrange(50):
          self.videoProcess1.join( timeout=0.1 )
          if not self.videoProcess1.is_alive():
            print "BREAK", i
            break
          self.update()
        else:
          print "TIMEOUT", i
        self.videoQueue1, self.videoProcess1 = None, None
      self.confirmedConfig( "AT*CONFIG=%i,\"video:video_codec\",\"129\"\r" )
      if self.replayLog == None and self.videoProcess2 != None:
        print "finishing record file"
        for i in xrange(300):
          self.videoProcess2.join(timeout=0.1) # i.e. wait 30sec for download and termination
          if not self.videoProcess2.is_alive():
            break
          self.update()
        self.videoQueue2, self.videoProcess2 = None, None
      print "DONE"

  def parseNavData( self, data ):
    "parse data and update ARDrone2 state"
    if len(data) > 16:
      header =  struct.unpack_from("IIII", data)
      offset = struct.calcsize("IIII")
      self.ctrlAck = header[1]>>6 & 1
#      self.userEmergencyLanding = header[1]>>16 & 1  # this is rather user input
      self.userEmergencyLanding = header[1]>>31 & 1  # Emergency landing : (0) no emergency, (1) emergency
      #print hex(header[1])
#      if header[1]>>19 & 1:
#        print "ERROR_STATE_EMERGENCY_ANGLE_OUT_OF_RANGE"
#      if header[1]>>22 & 1:
#        print "ARDRONE_CUTOUT_MASK"
      self.collision = False
      while True:
        tag, size =  struct.unpack_from("HH", data, offset)
        if tag == NAVDATA_CKS_TAG:
          break
        if tag == NAVDATA_DEMO_TAG:
          v = parseDemoTag( data, offset )
          self.ctrlState = v['ctrl_state']
          self.battery = v['battery']
          if self.headingOffset == None:
            self.headingOffset = math.radians(90.0) # used to be "math.radians(v['psi']/1000.)" but first data are not sufficiently reliable
          self.heading = self.headingOffset - math.radians(v['psi']/1000.)
          self.vx = v['vx']/1000.0
          self.vy = -v['vy']/1000.0
          dx = self.dt * self.vx
          dy = self.dt * self.vy
          c = math.cos(self.heading)
          s = math.sin(self.heading)
          self.coord = (self.coord[0]+c*dx-s*dy, self.coord[1]+s*dx+c*dy, v['altitude']/1000.0)
          assert v['vz']== 0.0
          self.angleLR = math.radians(v['phi']/1000.)
          self.angleFB = math.radians(v['theta']/1000.)
          if max(abs(self.angleLR), abs(self.angleFB)) > COLLISION_LIMIT_ANGLE:
            self.collision = True
#          if self.acc:
#            print self.ctrlState, '|\t%0.3f\t%0.3f\t%0.3f\t%d\t%d\t%d' % (v['theta']/1000., v['phi']/1000., v['psi']/1000., self.acc[0], self.acc[1], self.acc[2])
#          print self.ctrlState, "(%.1f, %.1f, %.1f)" % self.coord
          viewlog.dumpPose( (self.coord[0], self.coord[1], self.heading) )

        if tag == NAVDATA_TIME_TAG:
          t = parseTimeTag( data, offset )
          prev = self.time
          if self.time == None or self.time <= t:
            self.time = t
          else:
            self.time = t + (1<<11) # this handles only one 35min overflow
          if prev:
            self.dt = self.time - prev
            if int(prev*10) != int(self.time*10):
              viewlog.dumpSamples([(self.coord[0], self.coord[1], self.heading)])

        if tag == NAVDATA_RAW_MEASURES_TAG:
          self.acc = parseRawMeasuresTag( data, offset )
          if max(abs(self.acc[0]-2000), abs(self.acc[1]-2000)) > COLLISION_LIMIT_ACC:
            self.collision = True

        if tag == NAVDATA_VISION_DETECT_TAG:
          (countTypes, x, y, height, width, dist, oriAngle, cameraSource) = parseVisionDetectTag( data, offset )
          self.visionTag = []
          for i in xrange(countTypes[0]):
            if countTypes[i+1]:
              self.visionTag.append( (x[i], y[i], height[i], width[i], dist[i], oriAngle[i], cameraSource[i]) )
          if self.visionTag:
            viewlog.dumpBeacon( (self.coord[0], self.coord[1]), index=3 ) # well I did not properly implement the colors :( -> index=3

        if tag == NAVDATA_ALTITUDE_TAG:
          self.altitudeData = parseAltitudeTag( data, offset )

        if tag == NAVDATA_MAGNETO_TAG:
          self.magneto = parseMagnetoTag( data, offset )
          self.compass = math.radians(self.magneto[12])
          viewlog.dumpCompass( self.compass )

        if tag == NAVDATA_PRESSURE_RAW_TAG:
          self.pressure = parsePressureRawTag( data, offset )

        if tag == NAVDATA_GPS_TAG:
          self.gps = parseGPSTag( data, offset )

        offset += size


  def wait( self, seconds ):
    startTime = self.time
    assert( startTime != None )
    while self.time-startTime < seconds:
      self.update()

  def at_ref(seq, takeoff, emergency=False):
    """
    Basic behaviour of the drone: take-off/landing, emergency stop/reset)

    Parameters:
    seq -- sequence number
    takeoff -- True: Takeoff / False: Land
    emergency -- True: Turn of the engines
    """
    p = 0b10001010101000000000000000000
    if takeoff:
        p += 0b1000000000
    if emergency:
        p += 0b0100000000
    at("REF", seq, [p]) 

  def reset( self ):
    print "!!! RESET !!!"
    p = 0b10001010101000000000000000000
    p += 0b0100000000
    self.update( "AT*REF=%i,"+str(p)+'\r' )  # Emergency True
    p = 0b10001010101000000000000000000
    self.update( "AT*REF=%i,"+str(p)+'\r' )  # Emergency False

  def takeoff( self, timeout=10.0, enabledCorrections = False ):
    print "TRIM ..."
    self.update( "AT*FTRIM=%i,\r" )
    self.wait(0.1)
    print "Taking off ..."
    startTime = self.time
    assert( startTime != None )
    assert( self.ctrlState != None )
    p = 0b10001010101000000000000000000
    p += 0b1000000000    
    while self.time-startTime < timeout:
      self.update( "AT*REF=%i,"+str(p)+'\r' )
      if self.ctrlState in [3,4]: # CTRL_FLYING=3, CTRL_HOVERING=4
        break
      if enabledCorrections and self.time-startTime > 1.0 and self.coord[2] > 0.2:
        invCoord = relCoord( self.coord[0], self.coord[1], self.heading )
        vx, vy = 0, 0
        limit = 0.2
        if invCoord[0] > limit:
          vx = 0.1
        if invCoord[0] < -limit:
          vx = -0.1
        if invCoord[1] > limit:
          vy = 0.1
        if invCoord[1] < -limit:
          vy = -0.1
        print math.degrees(self.heading), self.coord, invCoord, vx, vy
        self.moveXYZA(vx,vy,0,0)
    print "state %d at %.1fs ... DONE" % ( self.ctrlState, self.time-startTime )


  def land( self, timeout = 10.0 ):
    print "Landing ...", self.coord
    startTime = self.time
    assert( startTime != None )
    assert( self.ctrlState != None )
    p = 0b10001010101000000000000000000
    while self.time-startTime < timeout:
      self.update( "AT*REF=%i,"+str(p)+'\r' )
      if self.ctrlState == 2: # CTRL_LANDED=2
        break
    print "Landing at %.1fs ... DONE" % ( self.time-startTime )

  def hover( self, time ):
    print "Hover..."
    startTime = self.time
    assert( startTime != None )
    while self.time-startTime < time:
      self.update("AT*PCMD=%i,0,0,0,0,0\r")

  def movePCMD( self, leftRight, frontBack, upDown, turn ):
    self.update("AT*PCMD=%i,1,"+ "%d,%d,%d,%d"%(f2i(leftRight),f2i(frontBack),f2i(upDown),f2i(turn)) + "\r")

  def moveForward( self, dist, timeout=3.0 ):
    print "Forward ...", dist
    refX = self.coord[0]
    refY = self.coord[1]
    startTime = self.time
    while self.time-startTime < timeout:
      self.movePCMD( 0.0, -self.speed, 0.0, 0.0 )
      if distance( (refX, refY), self.coord ) > dist:
        print "DIST REACHED (%.2f, %.2f, %.2f)" % self.coord
        break
    print distance( (refX, refY), self.coord ), self.time-startTime

  def moveBackward( self, dist, timeout=3.0 ):
    print "Backward ...", dist
    refX = self.coord[0]
    refY = self.coord[1]
    startTime = self.time
    while self.time-startTime < timeout:
      self.movePCMD( 0.0, self.speed, 0.0, 0.0 )
      if distance( (refX, refY), self.coord ) > dist:
        print "DIST REACHED (%.2f, %.2f, %.2f)" % self.coord
        break
    print distance( (refX, refY), self.coord ), self.time-startTime

  def moveLeft( self, dist, timeout=3.0 ):
    print "Left ...", dist
    refX = self.coord[0]
    refY = self.coord[1]
    startTime = self.time
    while self.time-startTime < timeout:
      self.movePCMD( -self.speed, 0.0, 0.0, 0.0 )
      if distance( (refX, refY), self.coord ) > dist:
        print "DIST REACHED (%.2f, %.2f, %.2f)" % self.coord
        break
    print distance( (refX, refY), self.coord ), self.time-startTime

  def moveRight( self, dist, timeout=3.0 ):
    print "Right ...", dist
    refX = self.coord[0]
    refY = self.coord[1]
    startTime = self.time
    while self.time-startTime < timeout:
      self.movePCMD( self.speed, 0.0, 0.0, 0.0 )
      if distance( (refX, refY), self.coord ) > dist:
        print "DIST REACHED (%.2f, %.2f, %.2f)" % self.coord
        break
    print distance( (refX, refY), self.coord ), self.time-startTime

  def moveUp( self, distance ):
    print "Up ...", distance
    time = distance*1.0 # no idea of scaling
    startTime = self.time
    while self.time-startTime < time:
      self.movePCMD( 0.0, 0.0, self.speed, 0.0 )

  def moveDown( self, distance ):
    print "Down ...", distance
    time = distance*1.0 # no idea of scaling
    startTime = self.time
    while self.time-startTime < time:
      self.movePCMD( 0.0, 0.0, -self.speed, 0.0 )
    print "timeout", self.time-startTime

  def turnLeft( self, angle ):
    print "Turn left ...", angle
    time = angle*1.0 # no idea of scaling
    startTime = self.time
    while self.time-startTime < time:
      self.movePCMD( 0.0, 0.0, 0.0, -self.speed )

  def turnRight( self, angle ):
    print "Turn right ...", angle
    time = angle*1.0 # no idea of scaling
    startTime = self.time
    while self.time-startTime < time:
      self.movePCMD( 0.0, 0.0, 0.0, self.speed )


  def moveDiagonal( self, dist, timeout=2.0 ):
    print "Diagonal FL ...", dist
    refX = self.coord[0]
    refY = self.coord[1]
    startTime = self.time
    while self.time-startTime < timeout:
      self.movePCMD( -self.speed/2.0, -self.speed/2.0, 0.0, 0.0 )
      if distance( (refX, refY), self.coord ) > dist:
        print "DIST REACHED"
        break
    print distance( (refX, refY), self.coord ), self.time-startTime

  def moveXYZA( self, vx, vy, vz, va ):
    "make one step update with proper orientation"
    self.movePCMD( -vy, -vx, vz, -va )


  def navTagFB( self, timeout ):
    "make sure you are above tag in forward/backward direction"
    desiredScrY = 233 # TODO
    limit = 100 # TODO
    lostLimit = 100 # TODO
    lostCount = 0
    spd = self.speed/2.0
    self.update("AT*PCMD=%i,0,0,0,0,0\r") # one update with hover
    startTime = self.time
    while self.time-startTime < timeout:
      vision = self.visionTag
      if not vision:
        lostCount += 1
        if lostCount > lostLimit:
          print "LOST navTagFB", lostCount
          return False
        self.update("AT*PCMD=%i,0,0,0,0,0\r") # or repeat last command??
        continue
      lostCount = 0
      scrY = self.visionTag[0][1]
      print "diffY", desiredScrY - scrY
      if abs(desiredScrY - scrY) < limit:
        print vision
        self.update("AT*PCMD=%i,0,0,0,0,0\r")
        return True
      if scrY < desiredScrY:
        self.movePCMD( 0.0, -spd, 0.0, 0.0 ) # moveForward
      else:
        self.movePCMD( 0.0, spd, 0.0, 0.0 )
    print vision
    print "TIMEOUT"
    return None # ??

  def navTagLR( self, timeout ):
    "make sure you are above tag in left/right direction"
    desiredScrX = 300 # TODO
    limit = 100 # TODO
    lostLimit = 100 # TODO
    lostCount = 0
    spd = self.speed/2.0
    self.update("AT*PCMD=%i,0,0,0,0,0\r") # one update with hover
    startTime = self.time
    while self.time-startTime < timeout:
      vision = self.visionTag
      if not vision:
        lostCount += 1
        if lostCount > lostLimit:
          print "LOST navTagLR", lostCount
          return False
        self.update("AT*PCMD=%i,0,0,0,0,0\r") # or repeat last command??
        continue
      print vision
      lostCount = 0
      scrX = self.visionTag[0][0]
      print "diffX", desiredScrX - scrX
      if abs(desiredScrX - scrX) < limit:
        self.update("AT*PCMD=%i,0,0,0,0,0\r")
        return True
      if scrX < desiredScrX:
        print "LEFT"
        self.movePCMD( -spd, 0.0, 0.0, 0.0 ) # moveLeft
      else:
        print "RIGHT"
        self.movePCMD( spd, 0.0, 0.0, 0.0 )
    print "TIMEOUT"
    return None # ??

  def navTagAngle( self, timeout ):
    "make sure you are above tag in orientation"
    desiredScrAngle = 0.0 # or 270
    limit = 5 # TODO
    lostLimit = 100 # TODO
    lostCount = 0
    spd = self.speed/2.0
    self.update("AT*PCMD=%i,0,0,0,0,0\r") # one update with hover OR update only?
    startTime = self.time
    while self.time-startTime < timeout:
      vision = self.visionTag
      if not vision:
        lostCount += 1
        if lostCount > lostLimit:
          print "LOST navTagAngle", lostCount
          return False
        self.update("AT*PCMD=%i,0,0,0,0,0\r") # or repeat last command??
        continue
      lostCount = 0
      scrAngle = self.visionTag[0][5]
      diff = desiredScrAngle - scrAngle
      if diff > 180:
        diff -= 360
      elif diff < -180:
        diff += 360
#      print "diff", diff, limit, abs(diff) < limit
      if abs(diff) < limit:
        self.update("AT*PCMD=%i,0,0,0,0,0\r")
        print vision
        return True
      if diff < 0:
        self.movePCMD( 0.0, 0.0, 0.0, -spd ) # turnLeft
      else:
        self.movePCMD( 0.0, 0.0, 0.0, spd )
    print "TIMEOUT"
    print vision
    return None # ??





  def hoverRoundel0( self, time ):
    print "HoverRoundel..."
    startTime = self.time
#    self.confirmedConfig( "AT*CONFIG=%i,\"control:flying_mode\",\"2\"\r" ) # HOVER_ON_TOP_OF_ORIENTED_ROUNDEL=2
    self.confirmedConfig( "AT*CONFIG=%i,\"control:flying_mode\",\"3\"\r" ) # HOVER_ON_TOP_OF_ORIENTED_ROUNDEL=2
    assert( startTime != None )
    while self.time-startTime < time:
      self.update("AT*PCMD=%i,0,0,0,0,0\r")
    self.confirmedConfig( "AT*CONFIG=%i,\"control:flying_mode\",\"0\"\r" ) # FREE Flight

  def hoverRoundel( self, time ):
    print "HoverRoundel..."
    startTime = self.time
    while self.time-startTime < time:
      self.navTagFB(0.1)
      self.navTagLR(0.1)
      self.navTagAngle(0.1)


  def flyToRoundel( self, timeout ):
    print "Fly to roundel ..."
    startTime = self.time
    while self.time-startTime < timeout:
      self.movePCMD( 0.0, -self.speed, 0.0, 0.0 )
      if self.visionTag:
        break
    print self.visionTag
#??    self.update("AT*PCMD=%i,0,0,0,0,0\r")
#    self.hoverRoundel( 1.0 )
    self.hover( 1.0 )

  def fly1meter( self, timeout ):
    print "Fly1meter"
    refX = self.coord[0]
    refY = self.coord[1]
    startTime = self.time
    while self.time-startTime < timeout:
      self.movePCMD( 0.0, -self.speed, 0.0, 0.0 )
      if distance( (refX, refY), self.coord ) > 1.0:
        print "DIST REACHED"
        break
    print distance( (refX, refY), self.coord ), self.time-startTime

  def flyToWall( self, vx, vy, timeout ):
    print "Fly to wall ..."
    startTime = self.time
    while self.time-startTime < timeout:
      self.movePCMD( -vy, -vx, 0.0, 0.0 )
      if self.collision:
        print "WALL HIT", math.degrees(self.angleLR), math.degrees(self.angleFB), self.acc
        break
    print "Fly END", self.time-startTime

  def flyToAltitude( self, altitude, timeout=3.0 ):
    print "Fly to altitude", altitude, "from", self.coord[2]
    startTime = self.time
    if self.coord[2] > altitude:
      while self.coord[2] > altitude and self.time-startTime < timeout:
        self.movePCMD( 0.0, 0.0, -self.speed, 0.0 )
    else:
      while self.coord[2] < altitude and self.time-startTime < timeout:
        self.movePCMD( 0.0, 0.0, self.speed, 0.0 ) # move up
    print "DONE at altitude", self.coord[2], "time", self.time-startTime

  def flyAtHeight( self, altitude, dist, timeout=20.0 ):
    print "flyAtHeight", altitude, dist, timeout 
    refX = self.coord[0]
    refY = self.coord[1]
    zTolerance = 0.5 # +/- no action
    startTime = self.time
    while self.time-startTime < timeout:
      altSonar, altVision = altitude, altitude
      if self.altitudeData != None:
        altVision = self.altitudeData[0]/1000.0
        altSonar = self.altitudeData[3]/1000.0
#      print altSonar, altVision
      if max(altSonar, altVision) < altitude-zTolerance:
        self.moveXYZA( self.speed/2.0, 0.0, self.speed, 0.0 )
      elif min(altSonar, altVision) > altitude+zTolerance:
        self.moveXYZA( self.speed/2.0, 0.0, -self.speed, 0.0 )
      else:
        self.moveXYZA( self.speed, 0.0, 0.0, 0.0 )
      if distance( (refX, refY), self.coord ) > dist:
        print "DIST REACHED"
        break


  def halt( self ):
    print "halt()", self.coord
    print "Battery:", self.battery
    if self.sonar != None:
      self.sonar.requestStop()
      self.sonar.join()
      self.sonar.close()

def testCfg( drone ):
  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # TCP
  print "connecting ..."
  s.connect((HOST, CONTROL_PORT))
#  s.bind( ('',CONTROL_PORT) )  
  print "connected"
  drone.update("AT*CTRL=%i,4,0\r") # CFG_GET_CONTROL_MODE=4
  for i in xrange(100):
    drone.update()
  f = open("config.txt","wb")
  for i in xrange(10):
    drone.update()
    print i
    data = s.recv(1024)
    if data.endswith('\0'):
      data = data[:-1]
      f.write(data)
      f.flush()
      break
    print repr(data)
    f.write(data)
    f.flush()
    if data.endswith('\0'):
      break
  f.close()
  s.close()

def testVideo0( replayLog ):
  print "testVideo()"
  drone = ARDrone2( replayLog )
  
  drone.confirmedConfig( "AT*CONFIG=%i,\"video:video_codec\",\"136\"\r" )
#  drone.confirmedConfig( "AT*CONFIG=%i,\"video:video_codec\",\"129\"\r" )
#  drone.confirmedConfig( "AT*CONFIG=%i,\"video:video_codec\",\"131\"\r" )
  drone.wait(1.0)
#  testCfg( drone )
  #return # hack

  dualMode = True
  print "connecting video ... dualMode = ", dualMode
  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # TCP
  s.connect((HOST, VIDEO_PORT))
  f = open( datetime.datetime.now().strftime("video_%y%m%d_%H%M%S.bin"), "wb" )
  if dualMode:
    s2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # TCP
    s2.connect((HOST, VIDEO_RECORDER_PORT))
    f2 = open( datetime.datetime.now().strftime("video_rec_%y%m%d_%H%M%S.bin"), "wb" )
  
  for i in xrange(100):
    drone.update()
    data = s.recv(10240)
    f.write(data)
    f.flush()
    if dualMode:
      data = s2.recv(10240)
      f2.write(data)
      f2.flush()
  f.close()
  s.close()
  if dualMode:
    f2.close()
    s2.close()

def testVideo1( replayLog ):
  print "testVideo()"
  drone = ARDrone2( replayLog )
  
  drone.confirmedConfig( "AT*CONFIG=%i,\"video:video_codec\",\"136\"\r" )
#  drone.confirmedConfig( "AT*CONFIG=%i,\"video:video_codec\",\"129\"\r" )
#  drone.confirmedConfig( "AT*CONFIG=%i,\"video:video_codec\",\"131\"\r" )
  drone.wait(1.0)

  print "multiprocess implementation"
  q = Queue()
  p = Process(target=logVideoStream, args=((HOST, VIDEO_PORT), datetime.datetime.now().strftime("video_%y%m%d_%H%M%S.bin"), q,))
  p.start()
  for i in xrange(200):
    drone.update()
  print "sending END ..."
  q.put("The END")
  print "waiting for termination ..."
  p.join()
  print "DONE"


def testVideo( replayLog ):
  print "testVideo()"
  drone = ARDrone2( replayLog )
  if replayLog == None:
    drone.startVideo()
  drone.wait(2.0)
  drone.confirmedConfig( "AT*CONFIG=%i,\"video:video_channel\",\"0\"\r" ) # HORTical view
  drone.wait(2.0)
  drone.confirmedConfig( "AT*CONFIG=%i,\"video:video_channel\",\"1\"\r" ) # VERTical view
  drone.wait(2.0)
  if replayLog == None:
    drone.stopVideo()


def test0( drone ):
  drone.hover(1.0)

def test1( drone ):
  drone.flyAtHeight( altitude=1.5, dist=10.0, timeout=20.0 )
#  print "ROTATE"
#  speed = 0.2
#  for i in xrange(200):
#    drone.moveXYZA( 0, 0, 0, speed )
#  print drone.navTagAngle(5.0)
#  print drone.navTagFB(2.0)
#  print drone.navTagLR(2.0)
#  drone.hoverRoundel(2.0)
#  drone.flyToAltitude( 2.0, timeout=10.0 )
#  drone.moveForward(10.0, timeout=20.)
#  drone.hover(1)
#  drone.moveLeft(1.7)
  drone.hover(1)

#  drone.moveBackward(0.75)
#  drone.hover(1)
#  drone.moveDiagonal(0.5)
#  drone.hover(1)

def test2( drone ):
  drone.moveRight(0.75)
  drone.hover(1)
  drone.moveLeft(0.75)
  drone.hover(1)

def test3( drone ):
  drone.moveUp(1)
  drone.hover(1.0)
  drone.moveDown(1)
  drone.hover(1.0)

def test4( drone ):
  drone.fly1meter(2.0)
  drone.hover(1.0)

def testSqr( drone ):
#  drone.moveUp(1)
  s = 2.0
  drone.moveForward(s)
  drone.hover(1.5)
  drone.moveLeft(s)
  drone.hover(1.5)
  drone.moveBackward(s)
  drone.hover(1.5)
  drone.moveRight(s)
  drone.hover(1.5)
#  drone.moveDown(1)

def test5( drone ):
  drone.confirmedConfig( "AT*CONFIG=%i,\"control:flying_mode\",\"3\"\r" ) # HOVER_ON_TOP_OF_ORIENTED_ROUNDEL=2
  drone.moveForward(2.5, timeout=5.0)
  drone.hover(1)
  drone.confirmedConfig( "AT*CONFIG=%i,\"control:flying_mode\",\"0\"\r" ) # FREE Flight

def testRoundel( drone ):
  drone.flyToRoundel(timeout=1.0)

def digitalEight( drone ):
  print "digitalEight"
  w = 1.8 # 2.6m
  h = 1.8 # 2.6m
  t = 1.5
  drone.moveForward(0.5) # start
  drone.hover(t)
  for i in xrange(10):
    drone.moveLeft(h)
    drone.hover(t)
    drone.moveLeft(h)
    drone.hover(t)
    drone.moveBackward(w)
    drone.hover(t)
    drone.moveLeft(h)
    drone.hover(t)
    drone.moveForward(w)
    drone.hover(t)
    drone.moveRight(h)
    drone.hover(t)
    drone.moveBackward(w)
    drone.hover(t)
    drone.moveRight(h)
    drone.hover(t)
    drone.moveRight(h)
    drone.hover(t)
    drone.moveForward(w)
    drone.hover(t)



def digitalEight2( drone ):
  print "digitalEight2"
  w = 1.7 # 2.6m
  h = 1.7 # 2.6m
  t = 3.0
  drone.moveForward(0.5) # start
  drone.hover(t)
  for i in xrange(10):
    drone.moveLeft(h)
    drone.hoverRoundel(t)
    drone.moveLeft(h)
    drone.hoverRoundel(t)
    drone.moveBackward(w)
    drone.hoverRoundel(t)
    drone.moveLeft(h)
    drone.hoverRoundel(t)
    drone.moveForward(w)
    drone.hoverRoundel(t)
    drone.moveRight(h)
    drone.hoverRoundel(t)
    drone.moveBackward(w)
    drone.hoverRoundel(t)
    drone.moveRight(h)
    drone.hoverRoundel(t)
    drone.moveRight(h)
    drone.hoverRoundel(t)
    drone.moveForward(w)
    drone.hoverRoundel(t)


def testTurn( drone ):
  drone.turnLeft(3)
  drone.hover(1.0)
  drone.moveForward(1.0)
  drone.hover(1.0)


def ver0( drone ):
  s = 1.5
  drone.moveForward(s)
  drone.hover(1.5)
  drone.moveLeft(s)
  drone.hover(1.5)
  drone.moveLeft(s)
  drone.hover(1.5)
  drone.moveForward(s)
  drone.hover(1.5)
  drone.moveRight(s)
  drone.hover(1.5)
  drone.moveBackward(s)
  drone.hover(1.5)
  drone.moveBackward(s)
  drone.hover(1.5)
  drone.moveRight(s)
  drone.hover(1.5)

def ver1( drone ):
  for i in xrange(2):
    ver0( drone )

def testHitWall( drone ):
  speed = 0.2/5.
  for i in xrange(1):
    drone.flyToWall( speed, 0.0, timeout=3.0 )
    drone.moveBackward( 0.1, timeout=0.2 )
    drone.flyToWall( -speed, 0.0, timeout=3.0 )
    drone.moveForward( 0.1, timeout=0.2 )

def flyRobotemRovneVer0( drone ):
  drone.flyAtHeight( altitude=1.5, dist=100.0, timeout=200.0 )


def flyRobotemRovneVer1( self ):
    "fake function with self"
    MIN_HEADING_RADIUS = 1.0 # fly that distance without angle correction
    altitude, dist, timeout = 1.5, 200.0, 300.0
    desiredHeading = math.radians(145+5.71) #0.0 # TODO based on compass/test measurements
    print "flyAtHeight", altitude, dist, timeout, math.degrees(desiredHeading)
    refX = self.coord[0]
    refY = self.coord[1]
    line = Line( (refX, refY), (refX+math.cos(desiredHeading)*dist, refY+math.sin(desiredHeading)*dist) )
    zTolerance = 0.1 #0.5 # +/- no action
    yTolerance = 0.5
    angleTolerance = math.radians(5)
    speedUpDown = 2*self.speed
    speedLeftRight = self.speed/2.0
    speedAngle = self.speed
    startTime = self.time
    while self.time-startTime < timeout:
      altSonar, altVision = altitude, altitude
      if self.altitudeData != None:
        altVision = self.altitudeData[0]/1000.0
        altSonar = self.altitudeData[3]/1000.0
      speedXYZA = [ self.speed, 0, 0, 0 ]
#      print "sonar/vision", altSonar, altVision
      if max(altSonar, altVision) < altitude-zTolerance:
        speedXYZA = [ self.speed/2.0, 0.0, speedUpDown, 0.0 ]
      elif min(altSonar, altVision) > altitude+zTolerance:
        speedXYZA = [ self.speed/2.0, 0.0, -speedUpDown, 0.0 ]

      offsetY = line.signedDistance( (self.coord[0], self.coord[1]) )
#      print "offsetY", offsetY, math.degrees(self.heading)

      if offsetY > yTolerance:
        speedXYZA[1] = -speedLeftRight
      if offsetY < -yTolerance:
        speedXYZA[1] = speedLeftRight

      if distance( (refX, refY), self.coord ) > MIN_HEADING_RADIUS:
        headingDiff = normalizeAnglePIPI( self.heading-desiredHeading )
        if headingDiff > angleTolerance:
          speedXYZA[3] = -speedAngle
        if headingDiff < -angleTolerance:
          speedXYZA[3] = speedAngle

#      print self.coord, math.degrees(self.heading), speedXYZA
      self.moveXYZA( *tuple(speedXYZA) )

      if distance( (refX, refY), self.coord ) > dist:
        print "DIST REACHED"
        break

def calibrate( self, timeout=10.0 ):
    self.update( "AT*CALIB=%i,0" )
    startTime = self.time
    while self.time-startTime < timeout:
      self.update()

def homologation0( self, desiredSpeed = 0.5, timeout=10.0 ):
  "move slowly forward and stop in front of obstacle"
  print "HOMOLOGATION"
  K = 0.01
  limit = 0.2
  speed = 0.0
  startTime = self.time
  while self.time-startTime < timeout:
    self.moveXYZA( speed, 0, 0, 0 )
    err = desiredSpeed - self.vx
    speed = K*err
    if speed > limit:
      speed = limit
    if speed < -limit:
      speed = -limit
#    print self.vx, speed

def homologation( self, desiredSpeed = 1.0, timeout=10.0 ):
  "move slowly forward and stop in front of obstacle"
  print "HOMOLOGATION2"
  startTime = self.time
  while self.time-startTime < timeout:
    if self.vx > desiredSpeed:
      self.moveXYZA( 0, 0, 0, 0 )
    else:
      self.moveXYZA( self.speed, 0, 0, 0 )


def ROStep( self, desiredSpeed = 1.0, desiredHeading = math.radians(100), dist = 20.0 ):
    "fake function with self"
    MIN_HEADING_RADIUS = 1.0 # fly that distance without angle correction
    MAX_CORRIDOR_OFFSET = 5.0
    IGNORE_SONAR_TIME_STEP = 0.5 # for transition between corridors
    MAX_OBSTACLE_SIZE = 2.0 # when to return to 0 corridor
    altitude, timeout = 1.5, 300.0
    corridorOffset = 0.0 # for simple collision avoidance
    corridorResetAt = 0.0
    corridorIgnoreTime = 0.0
    obstacleCounter = 0
    prevSonarIndex = None

    print "flyAtHeight", altitude, dist, timeout, math.degrees(desiredHeading)
    refX = self.coord[0]
    refY = self.coord[1]
    line = Line( (refX, refY), (refX+math.cos(desiredHeading)*dist, refY+math.sin(desiredHeading)*dist) )
    zTolerance = 0.1 #0.5 # +/- no action
    yTolerance = 0.5
    angleTolerance = math.radians(5)
    speedUpDown = 2*self.speed
    speedLeftRight = self.speed/2.0
    speedAngle = self.speed
    startTime = self.time
    while self.time-startTime < timeout:
      if self.sonarData != None and self.sonarData < 1.0:
        if prevSonarIndex != self.sonarIndex:
          prevSonarIndex = self.sonarIndex
          obstacleCounter += 1
          print "OBSTACLE HOVER", self.sonarData, obstacleCounter
        # print self.sonarData
        # wait in front of the obstacle
        #self.update("AT*PCMD=%i,0,0,0,0,0\r")
        self.moveBackward( dist=1.5, timeout=3.0 )
        sonarScan = scan( self )
        gapDir = sonarScan.maxGap( limit=2.0 )
        if gapDir != None:          
          turnToAngle( self, gapDir )
        continue

      d = distance( (refX, refY), self.coord )
      if self.sonarData != None and self.sonarData < 2.0:
        if prevSonarIndex != self.sonarIndex:
          prevSonarIndex = self.sonarIndex
          obstacleCounter += 1
          print self.sonarData, obstacleCounter

        if corridorOffset < MAX_CORRIDOR_OFFSET and self.time > corridorIgnoreTime:
          corridorOffset += 1.0
          corridorIgnoreTime = self.time + IGNORE_SONAR_TIME_STEP
          corridorResetAt = d + self.sonarData + MAX_OBSTACLE_SIZE
          print "COLLISION AT", d, self.sonarData, corridorResetAt
      elif d > corridorResetAt:
        obstacleCounter = 0
        corridorOffset = 0.0


      altSonar, altVision = altitude, altitude
      if self.altitudeData != None:
        altVision = self.altitudeData[0]/1000.0
        altSonar = self.altitudeData[3]/1000.0
      speed = self.speed
      if self.vx > desiredSpeed:
        speed = 0

      speedXYZA = [ speed, 0, 0, 0 ]
#      print "sonar/vision", altSonar, altVision
      if max(altSonar, altVision) < altitude-zTolerance:
        speedXYZA = [ speed/2.0, 0.0, speedUpDown, 0.0 ]
      elif min(altSonar, altVision) > altitude+zTolerance:
        speedXYZA = [ speed/2.0, 0.0, -speedUpDown, 0.0 ]

      offsetY = line.signedDistance( (self.coord[0], self.coord[1]) ) - corridorOffset
#      print "offsetY", offsetY, math.degrees(self.heading)

      if offsetY > yTolerance:
        speedXYZA[1] = -speedLeftRight
      if offsetY < -yTolerance:
        speedXYZA[1] = speedLeftRight

      if distance( (refX, refY), self.coord ) > MIN_HEADING_RADIUS:
        headingDiff = normalizeAnglePIPI( self.heading-desiredHeading )
        if headingDiff > angleTolerance:
          speedXYZA[3] = -speedAngle
        if headingDiff < -angleTolerance:
          speedXYZA[3] = speedAngle

#      print self.coord, math.degrees(self.heading), speedXYZA
      self.moveXYZA( *tuple(speedXYZA) )

      if distance( (refX, refY), self.coord ) > dist:
        print "DIST REACHED"
        break

def turnToAngle( self, desiredHeading, timeout = 10.0, refCoord=None, keepInPlace=False, func=None ):
  startTime = self.time
  if refCoord == None:
    refCoord = self.coord
  print "turnToAngle: refCoord", math.degrees(desiredHeading), refCoord
  while self.time-startTime < timeout:
    angle = normalizeAnglePIPI( desiredHeading - self.heading )
    if abs(angle) < math.radians(10):
      break
    vx, vy = 0, 0
    
    limit = 0.5
    if keepInPlace and distance( self.coord, refCoord ) > limit:
      speed = 0.1
      invCoord = relCoord( self.coord[0]-refCoord[0], self.coord[1]-refCoord[1], self.heading )
      size = math.hypot( invCoord[0], invCoord[1] )
      vx, vy = speed*invCoord[0]/size, speed*invCoord[1]/size
    if normalizeAnglePIPI( desiredHeading - self.heading ) > 0:
      self.moveXYZA( vx, vy, 0, self.speed )
    else:
      self.moveXYZA( vx, vy, 0, -self.speed )
    if func != None:
      func( self )
  else:
    print "turnToAngle: TIMEOUT", timeout
  self.hover(0.1) # stop rotation


class SonarScan:
  def __init__( self ):
    self.arr=[]
    self.prevIndex = None

  def update( self, drone ):
    if drone.sonarIndex != self.prevIndex:
      self.arr.append( (drone.heading, drone.sonarData) )
      self.prevIndex = drone.sonarIndex

  def maxGap( self, limit, widthLimit=None ):
    "bool array for given distance limit"
    arr = [(heading, dist < limit) for (heading, dist) in self.arr*2]
    start = 0
    maxGapSize = None
    ret = None
    while True:
      for i in xrange(start, len(arr)-1):
        if arr[i][1] and not arr[i+1][1]:
          break
      else:
        break    
      for j in xrange(i, len(arr)-1):
        if not arr[j][1] and arr[j+1][1]:
          break
      else:
        break # end reached
      gapSize = normalizeAnglePIPI( arr[j][0] - arr[i+1][0] )
      if gapSize < 0:
        gapSize += math.radians(360)
      if maxGapSize == None or gapSize > maxGapSize:
        ret = normalizeAnglePIPI( arr[i+1][0]+gapSize/2. )
        maxGapSize = gapSize
      start = j
    if widthLimit != None and maxGapSize != None:
      if maxGapSize < widthLimit:
        return None
    return ret

def scan( self ):
  "scan 360 degrees"
  desiredHeading = self.heading
  refCoord = self.coord
  sonarScan = SonarScan()
  for deg in [90,180,270,0]:
    turnToAngle( self, desiredHeading+math.radians(deg), refCoord=refCoord, keepInPlace=True, func=sonarScan.update )
  return sonarScan

def reachSonarHeight( self, altitude, timeout = 3.0 ):
  "controlled by timeout!"
  print "reachSonarHeight", altitude
  startTime = self.time
  zTolerance = 0.1
  speedUpDown = 2*self.speed
  while self.time-startTime < timeout:
      altSonar, altVision = altitude, altitude
      speed = 0.0
      if self.altitudeData != None:
        altVision = self.altitudeData[0]/1000.0
        altSonar = self.altitudeData[3]/1000.0
      speedXYZA = [ 0, 0, 0, 0 ]
#      print "sonar/vision", altSonar, altVision
      if max(altSonar, altVision) < altitude-zTolerance:
        speedXYZA = [ speed/2.0, 0.0, speedUpDown, 0.0 ]
      elif min(altSonar, altVision) > altitude+zTolerance:
        speedXYZA = [ speed/2.0, 0.0, -speedUpDown, 0.0 ]
      self.moveXYZA( *tuple(speedXYZA) )


def indicate( self ):
  "indicate waypoint reached"
  reachSonarHeight( self, 0.5 )
  reachSonarHeight( self, 2.5 )
  reachSonarHeight( self, 1.5 )


def ROVer1( self, desiredSpeed = 0.5 ):
  ROStep( self, desiredSpeed = desiredSpeed, desiredHeading = math.radians(0), dist = 5.0 )
#  turnToAngle( self, math.radians(0) )
#  ROStep( self, desiredSpeed = desiredSpeed, desiredHeading = math.radians(0), dist = 5.0 )

def ROVer2( self, dirDist, desiredSpeed = 1.0 ):
  "navigation for given list of waypoints"
  for desiredHeading, desiredDist in dirDist:
    turnToAngle( self, desiredHeading )
    ROStep( self, desiredSpeed=desiredSpeed, desiredHeading=desiredHeading, dist=desiredDist )
    #indicate( self )


def manualControl( drone ):
  "manual control via keys"
  speed = 0.2 # or drone.speed?
  speedUpDown = 0.4
  while True:
    key = drone.console()
    if key:
      if key == ' ':
        return
      if key == ('\xe0', 'H'): # arrow up
         drone.moveXYZA( 0, 0, speedUpDown, 0 )
      elif key == ('\xe0', 'P'): # down
        drone.moveXYZA( 0, 0, -speedUpDown, 0 )
      elif key == ('\xe0', 'K'): # left
         drone.moveXYZA( 0, 0, 0, speed )
      elif key == ('\xe0', 'M'): # right
         drone.moveXYZA( 0, 0, 0, -speed )
      elif key == 'q': # forward
         drone.moveXYZA( speed, 0, 0, 0 )
      elif key == 'z': # backward
         drone.moveXYZA( -speed, 0, 0, 0 )
      elif key == 'p': # right
         drone.moveXYZA( 0, -speed, 0, 0 )
      elif key == 'i': # left
         drone.moveXYZA( 0, speed, 0, 0 )
      elif key == 'o': # hover
         drone.hover( 0.1 )
      else:
        return
    else:
      drone.update()


#def airrace( replayLog, metaLog=None ):
#  "Robot Challenge 2013 - AirRace competitions" 
def roboorienteering( replayLog, metaLog=None, dirDist=None ):
  drone = ARDrone2( replayLog, metaLog=metaLog )
  drone.startVideo()
  if drone.userEmergencyLanding:
    drone.reset()
  try:
    drone.wait(1.0)
    drone.takeoff( enabledCorrections = False )
#    homologation( drone )
#    ROVer1( drone )
    ROVer2( drone, dirDist )
#    indicate( drone )
#    scan( drone )
#    flyRobotemRovneVer0( drone )
#    flyRobotemRovneVer1( drone )
#    calibrate( drone )
#    drone.turnRight( 20.0 )

#    drone.flyToAltitude( 2.5 )
#    drone.hoverRoundel( time=60.0 )


#    test0( drone )
#    test1( drone )
#    ver1( drone )
#    digitalEight( drone )
#    digitalEight2( drone )
#    drone.flyToWall( 0.2, 0.0, timeout=3.0)
#    testHitWall( drone )
#    drone.moveDown( 0.5 )
#    drone.flyToAltitude( 0.3 )
#    drone.hover(3.0)

#    drone.flyToAltitude( 0.5 )
#    drone.hover(1.0)
#    drone.moveForward(0.5)
#    drone.hover(1.0)
#    drone.flyToAltitude( 0.7 )
#    drone.hover(1.0)

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
#  dirDist = []
  dirDist = ( (0,1), (math.radians(90), 2), (math.radians(180), 2), (math.radians(-90), 2), )* 2
  if sys.argv[1] not in ["airrace", "iarc", "video", "replay", "dummy"]:
    dirDist = waypoints2dirDist( loadWaypoints( sys.argv[1] ) )
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
#        print drone.userEmergencyLanding
#          if drone.visionTag:
#            print drone.visionTag
      except EOFError:
        pass
  else:
#    airrace( replayLog=replayLog, metaLog=metaLog )
#  testCfg()
#    testVideo( replayLog )
    roboorienteering( replayLog=replayLog, metaLog=metaLog, dirDist=dirDist )

