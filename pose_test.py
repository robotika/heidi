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
    self.assertEqual( str(Pose(1,2,3)), str(Pose(1,2,3).sub(Pose(0,0,0)))  )
    self.assertEqual( str(Pose(10,1,math.radians(45)).sub(Pose(9,2,0))), str(Pose(1,-1,math.radians(45))) )
    A = Pose(1,2,0.3) # the base coordinate system
    B = Pose(4,5,0.6)
    C = A.add(B)
    self.assertEqual( str(B), str(C.sub(A)) )
    self.assertNotEqual( str(A), str(C.sub(B)) ) # not symertic operation, like for matrices

  def testTuple( self ):
    self.assertEqual( tuple(Pose(1,2,3)), (1,2,3) )

  def testCoord( self ):
    self.assertEqual( Pose(1,2,3).coord(), (1,2) )

  def testStripDiffBug( self ):
    A = Pose(1.03, -3.39, math.radians(-18)).add(Pose(-0.09, 0.04, math.radians(-14)))
    B = Pose(1.34, -3.51, math.radians(-21)).add(Pose(-0.03, -0.02, math.radians(-19)))
    self.assertEqual( str(B.sub(A)), "(0.40, 0.02, -8)" ) # orig: (0.82, -0.10, -7)

if __name__ == "__main__":
  unittest.main() 

