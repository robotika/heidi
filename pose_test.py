from pose import *
import unittest


class PoseTest( unittest.TestCase ): 
  def testStr( self ):
    self.assertEqual( str(Pose()), "(0.00, 0.00, 0)" )
    self.assertEqual( str(Pose(1.234, 5, math.radians(45))), "(1.23, 5.00, 45)" )

if __name__ == "__main__":
  unittest.main() 

