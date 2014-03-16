"""
  Strips Localisation for Air Race
"""
from pose import Pose

class StripsLocalisation:
  def __init__( self ):
    self.basePose = Pose()
  
  def filterPose( self, pose ):
    return pose.sub( self.basePose )

  def updateFrame( self, pose, frameStrips ):
    pass

