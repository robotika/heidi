from paveproxy import *
from pave_test import buildPacket
import unittest
from StringIO import StringIO


class PaVEProxyTest( unittest.TestCase ): 
  def testUsage( self ):
    header,payload = buildPacket("Hello World!")
    pp = PaVEProxy( StringIO( header+payload ).read )
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    s.settimeout( 0.1 )
    data = None
    try:
      data = s.recv(1024)
    except socket.timeout:
      pass
    self.assertEqual( data, payload )
    pp.term()





if __name__ == "__main__":
  unittest.main() 

