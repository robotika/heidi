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
    self.pathPose = None
    self.refIndex = 0
    self.ref = []
    p = Pose()
    # PATH_TURN LEFT
    for i in xrange(10): # TODO fix number
      self.ref.append( p )
      p = p.add( Pose(0.4, 0.1, math.radians(18)) ) # TODO dist and angle
    # PATH_STRAIGHT
    for i in xrange(10): # TODO fix number
      self.ref.append( p )
      p = p.add( Pose(0.4, 0.0, 0) )

  
  def filterPose( self, pose ):
    return pose.sub( self.basePose )

  def diff2pathType( self, dx, dy, da ):
    da = normalizeAnglePIPI(da)
    if (0.25 < dx < 0.45) and abs(da) < math.radians(50):
      if abs(da) < math.radians(10):
        return PATH_STRAIGHT
      elif da > 0:
        return PATH_TURN_LEFT
      else:
        return PATH_TURN_RIGHT
    return None


  def evalDiff( self, p1, p2 ):
    (dx,dy,da) = p1.sub( p2 )
    return math.hypot(dx,dy)+abs(normalizeAnglePIPI(da)/math.radians(100))

  def bestMatch( self, pose, poseArr ):
    ret = None
    for i,p2 in enumerate(poseArr):
      if ret == None or self.evalDiff(pose,p2) < self.evalDiff(pose,poseArr[ret]):
        ret = i
    return ret

  def updateFrame( self, pose, frameStrips ):
    print pose, [str(p) for p in frameStrips]
    for i in xrange(len(frameStrips)):
      for j in xrange(len(frameStrips)):
        if i != j:
          (dx,dy,da) = frameStrips[i].sub(frameStrips[j])
          pt = self.diff2pathType( dx, dy, da )
          if pt:
            self.pathType = pt
            self.pathPose = pose.add( frameStrips[i] ) # also j should be OK
            break

    if len(frameStrips) == 1 and self.lastStripPose != None:
      sPose = pose.add( frameStrips[0] )
      (dx,dy,da) = sPose.sub( self.lastStripPose )
      pt = self.diff2pathType( dx, dy, da )
      if pt:
        self.pathType = pt
        self.pathPose = sPose

    for fs in frameStrips:
      sPose = pose.add( fs )
      if self.lastStripPose != None:
        print sPose.sub( self.lastStripPose )
      self.lastStripPose = sPose

    if len(frameStrips) > 0:
      sPose = pose.add( frameStrips[0] )
      self.refIndex = self.bestMatch( sPose, self.ref )

