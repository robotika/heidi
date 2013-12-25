#!/usr/bin/env python
"""
 functions for generation of logs for viewer
"""

import math
from os import sep

viewLogFile = None # is it automatically shared?

def dumpPose( pose ):
  if viewLogFile:
    viewLogFile.write( "RefPose 1 %.2f %.2f %.4f\n" % pose )

def dumpObstacles( obstacles ):
  if viewLogFile:
    viewLogFile.write( "Map\n" )
    for obj in obstacles:
      if len(obj) == 1: # single column
        x,y=obj[0]
        d = 0.025
        obj = ((x-d,y-d),(x+d,y-d),(x+d,y+d),(x-d,y+d),(x-d,y-d))
      for wall in zip(obj[:-1],obj[1:]):
        viewLogFile.write( "%f\t%f\t%f\t%f\n" % \
          (wall[0][0], wall[0][1], wall[1][0], wall[1][1]) )

def dumpSamples( samples ):
  if viewLogFile:
    viewLogFile.write( "Poses\n" );
    for s in samples:
      viewLogFile.write( "%f\t%f\t%f\n" % s )

def dumpCompass( angle ):
  if viewLogFile:
    viewLogFile.write( "Compass\t%f\n" % angle );

def dumpSharpsGeometry( sharpsGeometry ):
  if viewLogFile:
    viewLogFile.write( "Geometry" )
    for s in sharpsGeometry:
      viewLogFile.write( " %.2f %.2f %.4f" % s )
    viewLogFile.write( "\n" )

def dumpSharps( pose, sharp ):
  if viewLogFile and sharp:
    viewLogFile.write( "Ranger %.2f %.2f %.4f " % pose )
    for s in sharp:
      viewLogFile.write( "%f " % s )
    viewLogFile.write( "\n" )

def dumpPuck( coord ):
  if viewLogFile:
    viewLogFile.write( "Puck 1 %.2f %.2f\n" % coord )

def dumpBeacon( coord, index=None ):
  if viewLogFile:
    if index is None:
      viewLogFile.write( "Beacon 1 %.2f %.2f\n" % coord )
    else:
      viewLogFile.write( "Beacon 1 %.2f %.2f %d\n" % (coord[0], coord[1], index) )

def dumpCamera( filename, imgresult ):
  if viewLogFile:
    viewLogFile.write( "Image %s\n" % filename )
    viewLogFile.write( "ImageResult %s\n" % imgresult)

def viewLogExtension( robot, id, data ):
  if id == 0x80:
    if robot.localisation:
      dumpPose( robot.localisation.pose() )

def viewCompassExtension( robot, id, data ):
  if id == 0x80:
    if robot.compass and robot.localisation:
      pose = robot.localisation.pose()
      dumpCompass( pose[2] + math.radians( robot.compass/10.0 ) ) 

def viewPoseExtension( robot, id, data ):
  if id == 0x80:
    if robot.localisation:
      dumpSamples( [robot.localisation.pose()] ) 

#def viewCameraExtension( robot, id, data ):
#  assert( False ) # deprecated to be removed
#  if id == 'camera' and len(data)>=2 and data[1] != None:
#    log = sys.argv[2][:-18]  # TODO extra param for extensions (like for threads args?)
#    dumpCamera( log + data[1].split('/')[-1], data[0].strip() ) 

class ViewCameraExtension:
  def __init__( self, absPath ):
    self.absPath = absPath

  def viewCameraExtension( self, robot, id, data ):
    #TODO: Except of the absPath vers sys.argv[2][:-18] difference, this is same as viewlog.viewCameraExtension
    if id == 'camera' and len(data)>=2 and data[1] != None:
      if data[0] == None:
        dumpCamera( self.absPath + sep + data[1].split('/')[-1], "" )
      else:
        dumpCamera( self.absPath + sep + data[1].split('/')[-1], data[0].strip() )  

