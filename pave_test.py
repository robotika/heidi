from pave import *
import unittest

def buildPacket( payload ):
  return "PaVE" + struct.pack("BBHI", 3, 4, 12, 5) + "ABCDE"


class PaVETest( unittest.TestCase ): 
  def testEmpty( self ):
    p = PaVE()
    p.append( "" )
    self.assertEqual( p.extract(), "" )

  def testDummyCompletePacket( self ):
    p = PaVE()
    s = buildPacket( "ABCDE" )
    p.append( s )
    self.assertEqual( p.extract(), s )
    self.assertEqual( p.extract(), "" ) # nothing is left

  def testPartialPacket( self ):
    p = PaVE()
    s = buildPacket( "ABCDE" )
    p.append( s[:5] )
    self.assertEqual( p.extract(), "" ) # not ready yet
    p.append( s[5:] )
    self.assertEqual( p.extract(), s )
    self.assertEqual( p.extract(), "" ) # nothing is left

  def testCorruptedStart( self ):
    p = PaVE()
    s = buildPacket( "There will be corrupted few bytes in front ..." )
    p.append( "blabla" + s )
    self.assertEqual( p.extract(), s ) # skip non-PaVE part
 
  def testTwoPackes( self ):
    p = PaVE()
    s1 = buildPacket( "First packet" )
    s2 = buildPacket( "Second packet" )
    p. append( s1 + s2 )
    self.assertEqual( p.extract(), s1 )
    self.assertEqual( p.extract(), s2 )

  def testBadBytesBetweenTwoPackes( self ):
    p = PaVE()
    s1 = buildPacket( "First packet" )
    s2 = buildPacket( "Second packet" )
    p. append( s1 + "bad bytes" + s2 )
    self.assertEqual( p.extract(), s1 )
    self.assertEqual( p.extract(), s2 )


if __name__ == "__main__":
  unittest.main() 

