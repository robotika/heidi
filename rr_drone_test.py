from rr_drone import *
import unittest

class RRDoneTest( unittest.TestCase ): 
  def testApprox4pts( self ):
    poly = [(983, 579), (261, 579), (262, 578), (266, 576), (314, 553), 
        (358, 534), (370, 529), (448, 498), (481, 488), (587, 460), (749, 460),
        (817, 485), (838, 494), (920, 536), (944, 552)]
    trapezoid = [(a[0][0], a[0][1]) for a in approx4pts( np.int0(poly) )]
    self.assertEqual( len(trapezoid), 4 )
    self.assertTrue( trapezoid, [(983, 579), (261,579), (587, 460), (749, 460)] )

  def testTrapezoid2line( self ):
    self.assertEqual( trapezoid2line( [(983, 579), (261,579), (587, 460), (749, 460)] ),
        [((983+261)/2,579), ((587+749)/2, 460)] )

if __name__ == "__main__":
  unittest.main() 

