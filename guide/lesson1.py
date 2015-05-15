"""
  Lesson 1 - before you takeoff ...
"""
import sys
import os
import inspect
ARDRONE2_ROOT = os.path.realpath(os.path.abspath(os.path.join(os.path.split(inspect.getfile( inspect.currentframe() ))[0],"..")))
if ARDRONE2_ROOT not in sys.path:
    sys.path.insert(0, ARDRONE2_ROOT) # access to drone source without installation

from ardrone2 import ARDrone2

drone = ARDrone2()

drone.wait(10.0)      # active waiting for 10 seconds
print drone.battery
print drone.coord     # X, Y, Z coordinate

