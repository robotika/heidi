from striploc import *
import unittest


class StripsLocalisationTest( unittest.TestCase ): 
  def testUsage( self ):
    loc = StripsLocalisation()
    self.assertEqual( str(loc.basePose), "(0.00, 0.00, 0)" )
    loc.updateFrame( Pose(), [] )
    self.assertEqual( str(loc.filterPose( Pose() )), "(0.00, 0.00, 0)" )

if __name__ == "__main__":
  unittest.main() 

