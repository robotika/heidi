"""
  Pose - position and orientation
"""
import math

class Pose:
  def __init__( self, x=0.0, y=0.0, heading=0.0 ):
    self.x, self.y, self.heading = x, y, heading

  def __str__( self ):
    return "(%.2f, %.2f, %d)" % (self.x, self.y, math.degrees(self.heading))
