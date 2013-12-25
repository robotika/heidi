#!/usr/bin/python
"""
  Open telnet session to drone and get sonar readings
  usage:
       ./sonar.py <output file>
"""
import sys
from telnetlib import Telnet
from threading import Thread,Event,Lock 
import datetime

#SONAR_EXE = "/data/video/sonar-732-arm"
SONAR_EXE = "/data/video/sonar-130610-2034-arm"

class Sonar( Thread ):
  def __init__( self ):
    Thread.__init__(self) 
    self.setDaemon(True)
    self.lock = Lock()
    self.shouldIRun = Event()
    self.shouldIRun.set()
    self._data = None   
    self.session = None
    self.log = open( datetime.datetime.now().strftime("logs/sonar_%y%m%d_%H%M%S.log"), "w" )

  def run(self):
    if self.session == None:
      self.session = Telnet( '192.168.1.1' )
      self.checkDriver()
      self.startSonar()
      self.sonarIndex = 0
    while self.shouldIRun.isSet():
      self.update() 

  def data( self ):
    self.lock.acquire()
    xy = self._data
    self.lock.release()
    return xy

  def requestStop( self ):
    self.shouldIRun.clear() 

  def execute( self, cmd, verbose=True ):
    self.session.write( cmd + '\n' )
    ret = []
    while 1:
      line = self.session.read_until('\n', timeout=1.0).strip()
      self.log.write(str(line)+'\n')
      self.log.flush()
      if line == '#':
        break
      print line
      ret.append( line )
    return ret

  def checkDriver( self ):
    self.execute( "killall %s" % SONAR_EXE.split('/')[-1] )
    if self.execute( "ls /dev/ttyACM0" )[-1].find("No such file or directory") != -1:
      self.execute( "lsusb" )
      self.execute( "insmod /data/video/driver2/cdc-acm.ko" )
    self.execute( "stty -F /dev/ttyACM0 intr ^- quit ^- erase ^- kill ^- eof ^- start ^- stop ^- susp ^- rprnt ^- werase ^- lnext ^- flush ^-" )
    self.execute( "stty -F /dev/ttyACM0 -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke" )


  def startSonar( self ):
    self.session.write( SONAR_EXE + '\n' )


  def killSonar( self ):
    tmpSession = Telnet( '192.168.1.1' )
    tmpSession.write('killall %s\n' % SONAR_EXE.split('/')[-1])
    while 1:
      line = self.session.read_until('\n', timeout=1.0).strip()
      if line == '#':
        break
    tmpSession.close()


  def update( self ):
    line = self.session.read_until('\n', timeout=1.0).strip()
    self.log.flush()
    self.log.write(str(line)+'\n')
    s = line.split()
    if len(s) >= 2 and s[0] == "sonar:":
      self._data = self.sonarIndex, int(line.split()[1])
      self.log.write(str(self._data)+'\n')
      self.log.flush()
      self.sonarIndex += 1

  def close( self ):
    self.killSonar()
    self.session.close()

def testSonar( filename=None ):
  s = Sonar()
  for i in xrange(10):
    s.update()
    print s.data()
  s.close()

if __name__ == "__main__":
  if len(sys.argv) < 2:
    print __doc__
    sys.exit(2)
  testSonar( filename=sys.argv[1] )

