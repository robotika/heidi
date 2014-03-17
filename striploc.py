"""
  Strips Localisation for Air Race
"""
from pose import Pose
from airrace import PATH_TURN_LEFT, PATH_TURN_RIGHT, PATH_STRAIGHT
from ardrone2 import normalizeAnglePIPI # TODO refactoring
import math

class StripsLocalisation:
  def __init__( self ):
    self.basePose = Pose()
    self.lastStripPose = None
    self.pathType = PATH_TURN_LEFT

  
  def filterPose( self, pose ):
    return pose.sub( self.basePose )

  def diff2pathType( self, dx, dy, da ):
    da = normalizeAnglePIPI(da)
    if (0.35 < dx < 0.45) and abs(da) < math.radians(50):
      if abs(da) < math.radians(10):
        return PATH_STRAIGHT
      elif da > 0:
        return PATH_TURN_LEFT
      else:
        return PATH_TURN_RIGHT
    return None

  def updateFrame( self, pose, frameStrips ):
    print pose, [str(p) for p in frameStrips]
    for i in xrange(len(frameStrips)):
      for j in xrange(len(frameStrips)):
        if i != j:
          (dx,dy,da) = frameStrips[i].sub(frameStrips[j])
          pt = self.diff2pathType( dx, dy, da )
          if pt:
            self.pathType = pt
            break

    if len(frameStrips) == 1 and self.lastStripPose != None:
      sPose = pose.add( frameStrips[0] )
      (dx,dy,da) = sPose.sub( self.lastStripPose )
      pt = self.diff2pathType( dx, dy, da )
      if pt:
        self.pathType = pt

    for fs in frameStrips:
      sPose = pose.add( fs )
      if self.lastStripPose != None:
        print sPose.sub( self.lastStripPose )
      self.lastStripPose = sPose

