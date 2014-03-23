"""
  Strips Localisation for Air Race
"""
from pose import Pose
from airrace import PATH_TURN_LEFT, PATH_TURN_RIGHT, PATH_STRAIGHT
from ardrone2 import normalizeAnglePIPI # TODO refactoring
import math
import random

class StripsLocalisation:
  def __init__( self, numSamples = 10 ):
    self.basePose = Pose()
    self.lastStripPose = None
    self.lastFramePose = None # drone pose when the last image was taken
    self.pathType = PATH_TURN_LEFT
    self.pathPose = None
    self.refIndex = 0
    self.ref = []
    p = Pose()
    radius = 1.5
    crossing = 2.45
    angleStep = 0.412/radius #(math.pi-math.asin(radius/crossing))/8.
    distStep = 0.4
    # PATH_TURN LEFT
    for i in xrange(8):
      self.ref.append( p.add(Pose(0,0,-angleStep/2.0)) )
      p = p.add( Pose( distStep, 0, angleStep) )
    # PATH_STRAIGHT
    for i in xrange(10):
      self.ref.append( p )
      p = p.add( Pose(distStep, 0.0, 0) )
    # PATH_TURN RIGHT
    for i in xrange(16):
      self.ref.append( p.add(Pose(0,0,angleStep/2.0)) )
      p = p.add( Pose( distStep, 0, -angleStep) )
    # PATH_STRAIGHT
    for i in xrange(10):
      self.ref.append( p )
      p = p.add( Pose(distStep, 0.0, 0) )
    # PATH_TURN LEFT
    for i in xrange(8):
      self.ref.append( p.add(Pose(0,0,-angleStep/2.0)) )
      p = p.add( Pose( distStep, 0, angleStep) )

    self.random = random.Random(0).uniform
    cov=(1.2, 1.2, math.radians(25)) # TODO better estimate
    self.samples = [ Pose(*tuple([v+self.random(-c,c) for v,c in zip([0,0,0],cov)])) for i in range(numSamples)] 
    self.ssi = 0 # Selected Sample Index

  def evalMap( self, pose, frameStrips ):
    ret = 1.0
    foundAny = False
    for strip in self.ref:
      img = strip.sub(pose)
      if abs(img.x) < 0.5 and abs(img.y) < 0.7: # TODO correct camera view
        foundAny = True
        if len(frameStrips) > 0:
          val = min([self.evalDiff(img, fs, oriented=False) for fs in frameStrips])
#          if val < 0.1:
#            print val
          ret *= max(0.001, 1.0-val)**2 #math.exp( -val ) # is it good idea???
        else:
          # missing detection
          ret *= 0.1
    if not foundAny and len(frameStrips) > 0:
      ret *=0.05
    return ret

  def resample( self, weights ):
    "replace internal samples based on weights "
    weightSum = sum( weights )
    step = weightSum/len(self.samples)
    seed = self.random(0,step)
    newSet = [0]*len(self.samples)
    tmp = 0
    i = 0
    for s,w in zip(self.samples,weights):
      tmp += w
      while seed <= tmp:
        newSet[i] = s
        i += 1
        seed += step
    self.samples = newSet
    return weightSum

  def mclStep( self, poseStep, frameStrips ):
    dx, dy, da = poseStep
    varDist = 0.3
    varAngle = 0.1
    newSamples = []
    for s in self.samples:
      # TODO tune distribution
      newSamples.append( s.add( Pose(dx+dx*self.random(-varDist,varDist), dy+dy*self.random(-varDist,varDist), da+da*self.random(-varAngle,varAngle))))
    self.samples = newSamples 

    weights = []
    for s in self.samples:
      weights.append( self.evalMap( s, frameStrips ) )

    return self.resample( weights )




  def filterPose( self, pose ):
    return pose.sub( self.basePose )

  def diff2pathType( self, dx, dy, da ):
    da = normalizeAnglePIPI(da)
    if (0.25 < dx < 0.48) and abs(da) < math.radians(50):
      if abs(da) < math.radians(10):
        return PATH_STRAIGHT
      elif da > 0:
        return PATH_TURN_LEFT
      else:
        return PATH_TURN_RIGHT
    return None


  def evalDiff( self, p1, p2, oriented=True ):
    (dx,dy,da) = p1.sub( p2 )
    da = abs(normalizeAnglePIPI(da))
    if not oriented and da > math.pi/2.:
      da = math.pi - da
    return math.hypot(dx,dy)+da/math.radians(100)

  def bestMatch( self, pose, poseArr ):
    ret = None
    for i,p2 in enumerate(poseArr):
      if ret == None or self.evalDiff(pose,p2) < self.evalDiff(pose,poseArr[ret]):
        ret = i
    return ret

  def updateFrame( self, pose, frameStrips, verbose=False ):
    if verbose:
      print pose, [str(p) for p in frameStrips]
    updated = False
    for i in xrange(len(frameStrips)):
      for j in xrange(len(frameStrips)):
        if i != j:
          (dx,dy,da) = frameStrips[i].sub(frameStrips[j])
          pt = self.diff2pathType( dx, dy, da )
          if pt:
            self.pathType = pt
            self.pathPose = pose.add( frameStrips[i] ) # also j should be OK
            updated = True
            break

    if (len(frameStrips) >= 1 and not updated) and self.lastStripPose != None:
      for fs in frameStrips:
        for lsp in self.lastStripPose:
          sPose = pose.add( fs )
          (dx,dy,da) = sPose.sub( lsp )
          pt = self.diff2pathType( dx, dy, da )
          if pt:
            self.pathType = pt
            self.pathPose = sPose
            updated = True
            break

    if not updated and verbose:
      print "not updated"

    if len(frameStrips) > 0:
      self.lastStripPose = []
      for fs in frameStrips:
        sPose = pose.add( fs )
        if verbose and self.lastStripPose != None:
          print [str(sPose.sub( lsp )) for lsp in self.lastStripPose]
        self.lastStripPose.append( sPose )

    if len(frameStrips) > 0:
      sPose = pose.add( frameStrips[0] )
      self.refIndex = self.bestMatch( sPose, self.ref )

    ret = 0.0
    if self.lastFramePose:
      ret = self.mclStep( pose.sub( self.lastFramePose ), frameStrips )
    self.lastFramePose = pose
    return ret

