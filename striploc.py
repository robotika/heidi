"""
  Strips Localisation for Air Race
"""
from pose import Pose

class StripsLocalisation:
  def __init__( self ):
    self.basePose = Pose()
    self.lastStripPose = None

  
  def filterPose( self, pose ):
    return pose.sub( self.basePose )

  def updateFrame( self, pose, frameStrips ):
    for fs in frameStrips:
      sPose = pose.add( fs )
      if self.lastStripPose != None:
        print sPose.sub( self.lastStripPose )
      self.lastStripPose = sPose

