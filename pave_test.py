from pave import *
import unittest

def buildPacket( payload ):
  return "PaVE" + struct.pack("BBHI", 3, 4, 12, len(payload)), payload


class PaVETest( unittest.TestCase ): 
  def testEmpty( self ):
    p = PaVE()
    p.append( "" )
    self.assertEqual( p.extract(), ("","") )

  def testDummyCompletePacket( self ):
    p = PaVE()
    h,s = buildPacket( "ABCDE" )
    p.append( h+s )
    self.assertEqual( p.extract(), (h,s) )
    self.assertEqual( p.extract(), ("","") ) # nothing is left

  def testPartialPacket( self ):
    p = PaVE()
    h,s = buildPacket( "ABCDE" )
    p.append( (h+s)[:5] )
    self.assertEqual( p.extract(), ("","") ) # not ready yet
    p.append( (h+s)[5:] )
    self.assertEqual( p.extract(), (h,s) )
    self.assertEqual( p.extract(), ("","") ) # nothing is left

  def testCorruptedStart( self ):
    p = PaVE()
    h,s = buildPacket( "There will be corrupted few bytes in front ..." )
    p.append( "blabla" + h + s )
    self.assertEqual( p.extract(), (h,s) ) # skip non-PaVE part
 
  def testTwoPackes( self ):
    p = PaVE()
    h1,s1 = buildPacket( "First packet" )
    h2,s2 = buildPacket( "Second packet" )
    p. append( h1 + s1 + h2 + s2 )
    self.assertEqual( p.extract(), (h1,s1) )
    self.assertEqual( p.extract(), (h2,s2) )

  def testBadBytesBetweenTwoPackes( self ):
    p = PaVE()
    h1,s1 = buildPacket( "First packet" )
    h2,s2 = buildPacket( "Second packet" )
    p. append( h1 + s1 + "bad bytes" + h2 + s2 )
    self.assertEqual( p.extract(), (h1,s1) )
    self.assertEqual( p.extract(), (h2,s2) )


if __name__ == "__main__":
  unittest.main() 

