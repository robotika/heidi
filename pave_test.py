from pave import *
import unittest

class PaVE2Test( unittest.TestCase ): 
  def testEmpty( self ):
    p = PaVE()
    p.append( "" )
    self.assertEqual( p.extract(), "" )

  def testDummyCompletePacket( self ):
    p = PaVE()
    s = "PaVE" + struct.pack("BBII", 3, 4, 10, 5) + "ABCDE"
    p.append( s )
    self.assertEqual( p.extract(), s )
    self.assertEqual( p.extract(), "" ) # nothing is left


if __name__ == "__main__":
  unittest.main() 

