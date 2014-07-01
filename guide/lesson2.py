"""
  Lesson 2 - manual emergency stop
"""
import sys
sys.path.append('..') # access to drone source without installation
from ardrone2 import ARDrone2, ManualControlException

drone = ARDrone2()
try:
    drone.takeoff()
    drone.hover(3.0)
except ManualControlException, e:
    print "ManualControlException"
drone.land()

# vim: expandtab sw=4 ts=4

