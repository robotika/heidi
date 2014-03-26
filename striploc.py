"""
  Strips Localisation for Air Race
"""
from pose import Pose
from ardrone2 import normalizeAnglePIPI # TODO refactoring
from line import Line

import math

REF_CIRCLE_RADIUS = 1.4 # TODO measure in real arena!
REF_LINE_CROSSING_ANGLE = math.radians(50) # angle for selection of proper strip

LINE_OFFSET = 0.5 # asymetric line navigation
NUM_TRANSITION_STEPS = 5 # to switch from left/right orientation, also max limits
MAX_CIRCLE_OFFSET = LINE_OFFSET # for smooth transition, can be different



PATH_STRAIGHT = 'I'
PATH_TURN_RIGHT = 'R'
PATH_TURN_LEFT = 'L'


class StripsLocalisation:
  def __init__( self ):
    self.lastStripPose = None
    self.pathType = PATH_TURN_LEFT
    self.pathPose = None
    self.pathUpdated = False
    self.refCircle = None
    self.refLine = None
    self.countLR = 0


  def diff2pathType( self, dx, dy, da ):
    da = normalizeAnglePIPI(da)
    if (0.25 < dx < 0.48) and abs(da) < math.radians(50) and (abs(dy) < 0.25):
      if abs(da) < math.radians(10):
        return PATH_STRAIGHT
      elif da > 0:
        return PATH_TURN_LEFT
      else:
        return PATH_TURN_RIGHT
    return None


  def isSameStrip( self, pose1, pose2 ):
    dx,dy,da = pose1.sub(pose2)
    da = normalizeAnglePIPI(da)
    if abs(dx) < 0.2 and abs(dy) < 0.1 and abs(da) < math.radians(10):
      return True
    return False


  def updateFrame( self, pose, frameStrips, verbose=False ):
    if verbose:
      print pose, [str(p) for p in frameStrips]
    self.pathUpdated = False
    for i in xrange(len(frameStrips)):
      for j in xrange(len(frameStrips)):
        if i != j:
          (dx,dy,da) = frameStrips[i].sub(frameStrips[j])
          pt = self.diff2pathType( dx, dy, da )
          if pt:
            sPose = pose.add( frameStrips[i] )
            if pt != PATH_STRAIGHT or self.refLine == None or \
                  abs(normalizeAnglePIPI( self.refLine.angle - sPose.heading )) < REF_LINE_CROSSING_ANGLE:
              self.pathType = pt
              self.pathPose = sPose # also j should be OK
              self.pathUpdated = True
              break
            else:
              print "SKIPPED2"

    if (len(frameStrips) >= 1 and not self.pathUpdated) and self.lastStripPose != None:
      for fs in frameStrips:
        for lsp in self.lastStripPose:
          sPose = pose.add( fs )
          (dx,dy,da) = sPose.sub( lsp )
          pt = self.diff2pathType( dx, dy, da )
          if pt:
            if pt != PATH_STRAIGHT or self.refLine == None or \
                  abs(normalizeAnglePIPI( self.refLine.angle - sPose.heading )) < REF_LINE_CROSSING_ANGLE:
              self.pathType = pt
              self.pathPose = sPose
              self.pathUpdated = True
              break
            else:
              print "SKIPPED1"

    if not self.pathUpdated:
      if len(frameStrips) == 1 and self.lastStripPose != None:
        fs = frameStrips[0]
        sPose = pose.add( fs )
        for lsp in self.lastStripPose:
          if self.isSameStrip( sPose, lsp ):
            # the self.pathType is the same only the pose is updated
            self.pathPose = sPose
            self.pathUpdated = True
        if not self.pathUpdated:
          for lsp in self.lastStripPose:
            print sPose.sub(lsp)

    if len(frameStrips) > 0:
      self.lastStripPose = []
      for fs in frameStrips:
        sPose = pose.add( fs )
        if verbose and self.lastStripPose != None:
          print [str(sPose.sub( lsp )) for lsp in self.lastStripPose]
        self.lastStripPose.append( sPose )

    if self.pathType == PATH_TURN_LEFT:
      self.countLR = max( -NUM_TRANSITION_STEPS, self.countLR - 1 )
    if self.pathType == PATH_TURN_RIGHT:
      self.countLR = min( NUM_TRANSITION_STEPS, self.countLR + 1 )

    if self.pathPose:
      sPose = self.pathPose
      if self.pathType == PATH_TURN_LEFT:
        circCenter = sPose.add( Pose(0.0, REF_CIRCLE_RADIUS, 0 )).coord()
        self.refCircle = circCenter, REF_CIRCLE_RADIUS - MAX_CIRCLE_OFFSET*self.countLR/float(NUM_TRANSITION_STEPS)
      elif self.pathType == PATH_TURN_RIGHT:
        circCenter = sPose.add( Pose(0.0, -REF_CIRCLE_RADIUS, 0 )).coord()
        self.refCircle = circCenter, REF_CIRCLE_RADIUS + MAX_CIRCLE_OFFSET*self.countLR/float(NUM_TRANSITION_STEPS)
      else:
        self.refCircle = None
      if self.pathType == PATH_STRAIGHT:
        if self.refLine == None or abs(normalizeAnglePIPI( self.refLine.angle - sPose.heading )) < REF_LINE_CROSSING_ANGLE:
          offset = Pose()
          if self.countLR > 0:
            offset = Pose( 0, LINE_OFFSET, 0 )
          if self.countLR < 0:
            offset = Pose( 0, -LINE_OFFSET, 0 )
          sPose = sPose.add( offset )
          self.refLine = Line( (sPose.x-0.15*math.cos(sPose.heading), sPose.y-0.15*math.sin(sPose.heading)), 
                                   (sPose.x+0.15*math.cos(sPose.heading), sPose.y+0.15*math.sin(sPose.heading)) )
      else:
        self.refLine = None
    return self.pathUpdated


  def getRefCircleLine( self, pose ):
    "return best fitting circle/line for given position (which is now ignored)"
    return self.refCircle, self.refLine
