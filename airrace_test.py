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
    self.assertEqual( repr(allStripPoses( [((898, 257), (189, 53), -58)] )[0]), repr(Pose(s*(720/2-257), s*(-898+1280/2), math.radians(-32))) ) # not visible :(

  def testAllStripPoses( self ):
    # video_rec_140325_200709.bin 435/15
    asp = allStripPoses( [((318, 201), (134, 22), 82), ((293, 43), (90, 18), 82)], highResolution=False )
    self.assertEqual( len(asp), 2 )
    self.assertAlmostEqual( abs(asp[1].sub( asp[0] ).y), 0.01, 2 ) # straight line - should be less than 0.25 
    self.assertAlmostEqual( abs(asp[1].sub( asp[0] ).x), 0.36, 2 ) # ... too far individual was 0.51
    self.assertEqual( allStripPoses([]), [] )

  def testRemoveDuplicities( self ):
    self.assertEqual( removeDuplicities([]), [] )
    self.assertEqual( removeDuplicities( [((400, 162), (128, 20), -72), ((400, 161), (131, 24), -72)] ), [((400, 162), (128, 20), -72)] )
    self.assertEqual( removeDuplicities( [((209, 292), (133, 30), -87), ((201, 136), (95, 20), 77), ((198, 123), (143, 27), 78)] ),
      [((209, 292), (133, 30), -87), ((198, 123), (143, 27), 78)] )
    # MSER in [((599, 241), (89, 20), -18), ((353, 183), (78, 18), 90), ((352, 161), (128,23), 88)], frame 61?
    self.assertEqual( removeDuplicities([((599, 241), (89, 20), -18), ((353, 183), (78, 18), 90), ((352, 161), (128,23), 88)]), 
        [((599, 241), (89, 20), -18), ((352, 161), (128,23), 88)] )

  def testRemoveDuplicitiesMistakes( self ):
    # video_rec_140317_130554.bin : frame 14 - status quo
    #self.assertEqual( removeDuplicities( [((25, 169), (50, 109), 0), ((35, 139), (279, 71), -90), ((64, 338), (77, 22), -45), 
    #  ((65, 338), (82, 26), -45), ((52, 340), (111, 46), -24), ((214, 47), (54, 23), -78), ((216, 37), (76, 25), -78), 
    #  ((216, 37), (81, 28), -79), ((216, 48), (104, 39), -77), ((155, 196), (160, 26), -63), ((141, 188), (163, 62), -63)] ), 
    #  [((25, 169), (50, 109), 0), ((35, 139), (279, 71), -90), ((52, 340), (111, 46), -24), ((216, 48), (104, 39), -77), ((141, 188), (163, 62), -63)] )
    # filter single place (wrong example, on the border of image frame)
    #self.assertEqual( removeDuplicities( [((214, 47), (54, 23), -78), ((216, 37), (76, 25), -78), 
    #                                      ((216, 37), (81, 28), -79), ((216, 48), (104, 39), -77),] ), [((216, 48), (104, 39), -77)] )
    # well this is better example, with whole strip - WRONG SELECTION!
    self.assertEqual( removeDuplicities( [((155, 196), (160, 26), -63), ((141, 188), (163, 62), -63)] ), [((155, 196), (160, 26), -63)] )

if __name__ == "__main__":
  unittest.main() 

