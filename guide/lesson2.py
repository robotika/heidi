"""
  Lesson 2 - manual emergency stop
"""
from ardrone2 import ARDrone2, ManualControlException

drone = ARDrone2()
try:
    drone.takeoff()
    drone.hover(10.0)
except ManualControlException, e:
    print "ManualControlException"
drone.land()

# vim: expandtab sw=4 ts=4

