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

if __name__ == "__main__":
  unittest.main() 

