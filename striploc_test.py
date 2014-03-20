from striploc import *
import unittest


class StripsLocalisationTest( unittest.TestCase ): 
  def testUsage( self ):
    loc = StripsLocalisation()
    self.assertEqual( str(loc.basePose), "(0.00, 0.00, 0)" )
    loc.updateFrame( Pose(), [] )
    self.assertEqual( str(loc.filterPose( Pose() )), "(0.00, 0.00, 0)" )
    self.assertEqual( loc.pathType, PATH_TURN_LEFT )
    self.assertEqual( loc.pathPose, None )

  def testSingleFramePathType( self ):
    loc = StripsLocalisation()
    loc.updateFrame( Pose(), [ Pose(-0.17, 0.19, math.radians(36)), Pose(0.18, 0.37, math.radians(20))] )
    self.assertEqual( loc.pathType, PATH_TURN_RIGHT ) # note, that later this may fail do to incorrect basePose
    self.assertEqual( str(loc.pathPose), "(0.18, 0.37, 20)" ) # but could be also the other

  def testFollowingFrames( self ):
    "single strip detected in two following frames"
    loc = StripsLocalisation()
    loc.updateFrame( Pose(2.40, -5.13, math.radians(-74)), [Pose(-0.05, -0.04, math.radians(-8))] )
    loc.updateFrame( Pose(2.45, -5.53, math.radians(-83)), [Pose(-0.04, -0.12, math.radians(-6))] )
    self.assertEqual( loc.pathType, PATH_STRAIGHT )
    self.assertEqual( str(loc.pathPose), str(Pose(2.45, -5.53, math.radians(-83)).add(Pose(-0.04, -0.12, math.radians(-6)))) )

  def testEvalDiff( self ):
    loc = StripsLocalisation()
    self.assertAlmostEqual( loc.evalDiff( Pose(), Pose() ), 0.0, 5 )
    self.assertAlmostEqual( loc.evalDiff( Pose(1,2,0), Pose(4,6,0) ), 5.0, 5 )
    self.assertAlmostEqual( loc.evalDiff( Pose(0,0,math.radians(10)), Pose(0,0,math.radians(20)) ), 0.1, 5 )
    self.assertAlmostEqual( loc.evalDiff( Pose(0,0,math.radians(10)), Pose(0,0,math.radians(350)) ), 0.2, 5 )

  def testBestMatch( self ):
    loc = StripsLocalisation()
    self.assertEqual( loc.bestMatch( Pose(1,2,0), [Pose(), Pose(1,0,0), Pose(2,0,0)] ), 1 )

  def testReference( self ):
    turnStep = Pose( 0.4, 0.1, math.radians(18) ) # TODO proper values
    loc = StripsLocalisation()
    loc.updateFrame( Pose(0,0,0), [Pose(0,0,0)] ) # start strip
    self.assertEqual( loc.refIndex, 0 )
    loc.updateFrame( Pose(0.4,0.1,math.radians(18)), [Pose(0,0,0)] ) # 2nd strip on LEFT_TURN
    self.assertEqual( loc.refIndex, 1 )
    loc.updateFrame( turnStep.add(turnStep).add(turnStep), [Pose(0,0,0)] ) # 4th strip on LEFT_TURN
    self.assertEqual( loc.refIndex, 3 )

  def testTurnBug( self ):
    # video_rec_140318_185412.bin : picture 20
    loc = StripsLocalisation()
    loc.updateFrame( Pose(0.54, -3.06, math.radians(-19)), [Pose(0.01, 0.00, math.radians(-20))] )
    loc.updateFrame( Pose(0.97, -3.08, math.radians(-3)), [Pose(-0.13, -0.22, math.radians(-43)), Pose(0.09, -0.36, math.radians(-42))] )
    self.assertEqual( loc.pathType, PATH_STRAIGHT )

  def testEvalStrips( self ):
    loc = StripsLocalisation()
    self.assertAlmostEqual( loc.evalDiff( Pose(), Pose(0,0,math.radians(180)), oriented=False ), 0.0, 5 )

  def testMCL( self ):
    loc = StripsLocalisation( numSamples=2 )
    self.assertNotEqual( loc.samples[0], loc.samples[1] )
    loc.mclStep( Pose(1.0, 0, 0), [] )



if __name__ == "__main__":
  unittest.main() 
