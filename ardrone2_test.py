from ardrone2 import *
import unittest
import math

class Ardrone2Test( unittest.TestCase ): 
  def tupleAlmostEqual( self, a, b ):
    for x,y in zip(a,b):
      self.assertAlmostEqual(x,y, delta=0.00001, msg=str((a,b)))
  def testRelCoord( self ):
    self.assertEqual( relCoord( 0, 0, 0 ), (0,0) )
    self.assertEqual( relCoord( 0.5, 0.2, 0 ), (-0.5,-0.2) )
    self.tupleAlmostEqual( relCoord( 1, 0, math.radians(90) ), (0,1) )
    self.tupleAlmostEqual( 
        relCoord( 0.15139901571579156, 0.17409094203596967, math.radians(101.528)),
                (-0.14032242535469552, 0.18313638360865858) )

    self.tupleAlmostEqual( 
        relCoord( 0.15139901571579156, 0.17409094203596967, math.radians(90)),
                (-0.17409094203596967, 0.15139901571579156) )

  def testSonarScan( self ):
    s = SonarScan()
    self.assert_( s.maxGap( 1.0 ) == None ) # no info

    s.arr = [(0, 1.), (math.radians(90), 2.0)]
    self.assertAlmostEqual( s.maxGap( 1.5 ), math.radians(90) ) # the best known empty direction (not enough data)
    self.assert_( s.maxGap( 0.5 ) == None ) # no obstacles
    self.assert_( s.maxGap( 2.5 ) == None ) # no free space

    s.arr = [(math.radians(x),y) for (x,y) in [(0,1),(90,2),(180,1),(270,1)]]
    self.assertAlmostEqual( s.maxGap( 1.5 ), math.radians(90) ) # the best known empty direction (not enough data)

    s.arr = [(math.radians(x),y) for (x,y) in [(0,1),(45,2),(90,2),(180,1),(270,1)]]
    self.assertAlmostEqual( s.maxGap( 1.5 ), math.radians((45+90)/2.) ) # the best known empty direction (not enough data)

    s.arr = [(math.radians(x),y) for (x,y) in [(0,2),(10,2),(90,1),(180,1),(270,1),(350,2)]]
    self.assertAlmostEqual( s.maxGap( 1.5 ), math.radians(0) ) # singularity

    s.arr = [(math.radians(x),y) for (x,y) in [(0,1),(10,2),(20,1),(180,1),(270,2),(350,2)]]
    self.assertAlmostEqual( s.maxGap( 1.5 ), math.radians((270+350)/2.-360) ) # 2nd gap

    self.assert_( s.maxGap( 1.5, widthLimit=math.radians(90))==None )

if __name__ == "__main__":
  unittest.main() 

