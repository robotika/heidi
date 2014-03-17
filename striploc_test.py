from striploc import *
import unittest


class StripsLocalisationTest( unittest.TestCase ): 
  def testUsage( self ):
    loc = StripsLocalisation()
    self.assertEqual( str(loc.basePose), "(0.00, 0.00, 0)" )
    loc.updateFrame( Pose(), [] )
    self.assertEqual( str(loc.filterPose( Pose() )), "(0.00, 0.00, 0)" )
    self.assertEqual( loc.pathType, PATH_TURN_LEFT )

  def testSingleFramePathType( self ):
    loc = StripsLocalisation()
    loc.updateFrame( Pose(), [ Pose(-0.17, 0.19, math.radians(36)), Pose(0.18, 0.37, math.radians(20))] )
    self.assertEqual( loc.pathType, PATH_TURN_RIGHT ) # note, that later this may fail do to incorrect basePose

  def testFollowingFrames( self ):
    "single strip detected in two following frames"
    loc = StripsLocalisation()
    loc.updateFrame( Pose(2.40, -5.13, math.radians(-74)), [Pose(-0.05, -0.04, math.radians(-8))] )
    loc.updateFrame( Pose(2.45, -5.53, math.radians(-83)), [Pose(-0.04, -0.12, math.radians(-6))] )
    self.assertEqual( loc.pathType, PATH_STRAIGHT )

if __name__ == "__main__":
  unittest.main() 

