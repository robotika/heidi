#!/usr/bin/python
"""
  RoboOrienteering libraries
  usage:
       ro.py <waypoints file>
"""
import sys
import math
from line import distance
from route import Convertor

def angleTo( f, t ):
  if math.fabs(f[0]-t[0]) < 0.0001 and math.fabs(f[1]-t[1]) < 0.0001:
    return 0
  return math.atan2( t[1]-f[1], t[0]-f[0] ) 


def loadWaypoints( filename ):
  ret = []
  for line in open(filename):
    s = line.split()
    if len(s) == 3:
      print s
      ret.append( (float(s[2]), float(s[1]) ) )
  return ret 

def waypoints2dirDist( waypoints ):
  "convert GPS waypoints to desired heading and distance"
  conv = Convertor( waypoints[0] )
  ret = []
  for start,goal in zip(waypoints[:-1],waypoints[1:]):
    ret.append( (
        angleTo( conv.geo2planar(start), conv.geo2planar(goal) ),
        distance( conv.geo2planar(start), conv.geo2planar(goal) )
       ) )
  return ret

if __name__ == "__main__":
  if len(sys.argv) < 2:
    print __doc__
    sys.exit(2)
  filename = sys.argv[1]
  for dirDist in waypoints2dirDist( loadWaypoints( filename ) ):
    print math.degrees(dirDist[0]), dirDist[1]

