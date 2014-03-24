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
    turnStep = Pose( 0.4, 0.1, math.radians(15) ) # TODO proper values
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

  def testMultistripRobustness( self ):
    loc = StripsLocalisation( numSamples=1 )
    loc.updateFrame( Pose(2.33, -3.38, math.radians(69)), [Pose(-0.04, 0.03, math.radians(-2))] )
    loc.updateFrame( Pose(2.42, -3.02, math.radians(68)), [Pose(0.07, -0.06, math.radians(-4)), Pose(0.25, -0.47, math.radians(69))] )
    self.assertEqual( loc.pathType, PATH_STRAIGHT )

  def testFilterWrongStrips( self ):
    loc = StripsLocalisation( numSamples=1 )
    loc.pathType = PATH_TURN_RIGHT # just enforce different value than L
    loc.updateFrame( Pose(-0.69, -0.98, math.radians(252)), [Pose(0.07, 0.12, math.radians(10)), Pose(0.30, -0.48, math.radians(-61))] )
    loc.updateFrame( Pose(-0.71, -1.38, math.radians(268)), [Pose(-0.28, -0.69, math.radians(-77)), Pose(0.14, 0.13, math.radians(8))] )
    self.assertEqual( loc.pathType, PATH_TURN_LEFT )

  def testTrippleStripPreference( self ):
    # meta_140320_165035.log:270
    loc = StripsLocalisation( numSamples=1 )
    loc.updateFrame( Pose(3.30, 1.97, math.radians(257)), [Pose(-0.17, -0.28, math.radians(13)), Pose(0.12, -0.25, math.radians(27))] )
    loc.updateFrame( Pose(3.48, 1.43, math.radians(269)), [Pose(-0.20, -0.38, math.radians(33)), Pose(0.04, -0.23, math.radians(48)), Pose(0.29, 0.06, math.radians(49))] )
    self.assertEqual( loc.pathType, PATH_STRAIGHT )
    self.assertTrue( loc.pathUpdated )

  def testTransitionLR( self ):
    # meta_140320_165035.log:268 (similar case also frame=66)
    loc = StripsLocalisation( numSamples=1 )
    loc.updateFrame( Pose(3.03, 2.21, math.radians(269)), [Pose(-0.22, 0.10, math.radians(1)), Pose(0.15, 0.16, math.radians(16)), Pose(0.34, 0.80, math.radians(-32))] )
    self.assertEqual( loc.pathType, PATH_TURN_LEFT )
    self.assertTrue( loc.pathUpdated )

  def testIsSameStrip( self ):
    loc = StripsLocalisation( numSamples=1 )
    self.assertTrue( loc.isSameStrip( Pose(0,0,0), Pose(0.15,0.05,math.radians(1))))
    self.assertFalse( loc.isSameStrip( Pose(0,1.0,0), Pose(0.3,0.05,math.radians(1))))
    # real examples
    # 9:  0.186620639305 0.0527981159447 -0.81
    # 35: 0.170391659423 0.0771475958297 -2.136
    # 41: 0.156540370398 -0.00496615873208 4.462
    
    # FAILED:
    # 123: 0.14906722044 -0.118410346849 1.394
    # 174: 0.219441940085 0.0828970285437 0.937
    # 185: 0.152514448606 -0.100774758903 -1.427 (bad middle strip)
    # 234: 0.210778241227 -0.0182408657026 359.693
    

if __name__ == "__main__":
  unittest.main() 

