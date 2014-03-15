from airrace import *
import unittest


class AirRaceTest( unittest.TestCase ): 
  def testFilterRectanbles( self ):
    self.assertEqual( filterRectangles([]), [] )
    self.assertEqual( filterRectangles(
      [((817, 597), (10, 12), -26), ((223, 440), (319, 60), -88)]), 
      [((223, 440), (319, 60), -88)] )
    self.assertEqual( filterRectangles( [((298, 508), (58, 319), -15)] ), [((298, 508), (319, 58), 75)] )
    self.assertEqual( filterRectangles( [((982, 492), (29, 17), -45), ((951, 507), (60, 84), -69)] ), [] )

  def testStripPose( self ):
    s = 0.3/319.
    self.assertEqual( repr(stripPose( ((223, 440), (319, 60), -88) )), repr(Pose(s*(720/2-440), s*(-223+1280/2), math.radians(-2))) )
    self.assertEqual( repr(stripPose( ((298, 508), (319, 58), 75) )), repr(Pose(s*(720/2-508), s*(-298+1280/2), math.radians(15))) )
    s = 0.3/float(319/2)
    self.assertEqual( repr(stripPose( ((298/2, 508/2), (319/2, 58/2), 75), highResolution=False )), repr(Pose(s*((720/2-508)/2), s*((-298+1280/2)/2), math.radians(15))) )

  def testZScalingBug( self ):
    sWidth = 0.3/189.
    s = 0.05/53.
    self.assertEqual( repr(stripPose( ((898, 257), (189, 53), -58) )), repr(Pose(s*(720/2-257), s*(-898+1280/2), math.radians(-32))) )

  def testClassifyPath( self ):
    self.assertEqual( classifyPath( [Pose(0,0,0)] ), PATH_UNKNOWN )
    self.assertEqual( classifyPath( [Pose(0,0,0), Pose(0.3,0,0),] ), PATH_STRAIGHT )

    # reference frames from "video_rec_140225_200914.bin"
    poses = [stripPose(rec) for rec in [((968, 580), (271, 51), -16), ((305, 383), (295, 52), 90)]] # frame 270
    self.assertEqual( classifyPath( poses ), PATH_CROSSING )
    
    poses = [stripPose(rec) for rec in [((327, 551), (239, 43), 76), ((244, 235), (216, 43), 76)]] # frame 510
    self.assertEqual( classifyPath( poses ), PATH_STRAIGHT )

    poses = [stripPose(rec) for rec in [((713, 557), (200, 36), -84), ((778, 295), (210, 37), -68)]] # frame 750
    self.assertEqual( classifyPath( poses ), PATH_TURN_RIGHT )

    poses = [stripPose(rec) for rec in [((557, 507), (202, 35), -71), ((685, 259), (211, 37), -54)]] # frame 780
    self.assertEqual( classifyPath( poses ), PATH_TURN_RIGHT )

    poses = [stripPose(rec) for rec in [((473, 616), (187, 36), -79), ((562, 356), (206, 36), -63), ((717, 124), (211, 37), -48)]] # 1110
    self.assertEqual( classifyPath( poses ), PATH_TURN_RIGHT )

    poses = [stripPose(rec) for rec in [((532, 608), (203, 49), 68), ((337, 268), (279, 50), 53)]] # 330 src_cv2_140304_191305.log
    self.assertEqual( classifyPath( poses ), PATH_TURN_LEFT )

  def testClassifyPathBug( self ):
    # video_rec_140306_175854.bin
    # frame 630, mixed frames - too wide -> almost straight
    poses = [stripPose(rec) for rec in [((670, 526), (182, 32), 82), ((660, 282), (201, 43), 90)]]
    self.assertEqual( classifyPath( poses ), PATH_STRAIGHT )

if __name__ == "__main__":
  unittest.main() 

