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
    self.assertEqual( stripPose( ((223, 440), (319, 60), -88) ), (720/2-440, 223-1280/2, math.radians(-2)) )
    self.assertEqual( stripPose( ((298, 508), (319, 58), 75) ), (720/2-508, 298-1280/2, math.radians(15)) )

if __name__ == "__main__":
  unittest.main() 

