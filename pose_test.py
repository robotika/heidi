from pose import *
import unittest


class PoseTest( unittest.TestCase ): 
  def testStr( self ):
    self.assertEqual( str(Pose()), "(0.00, 0.00, 0)" )
    self.assertEqual( str(Pose(1.234, 5, math.radians(45))), "(1.23, 5.00, 45)" )

  def testAdd( self ):
    self.assertEqual( str(Pose(1,2,math.radians(90)).add( Pose(3,4,math.radians(45))) ),
        str(Pose( 1-4, 2+3, math.radians(90+45) )) )

  def testSub( self ):
    A = Pose(1,2,0.3)
    B = Pose(4,5,0.6)
    C = A.add(B)
    self.assertEqual( str(A), str(C.sub(B)) )
    self.assertNotEqual( str(B), str(C.sub(A)) ) # not symertic operation, like for matrices

if __name__ == "__main__":
  unittest.main() 

