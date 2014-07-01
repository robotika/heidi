"""
  Lesson 6 - record video from starting area
"""
import sys
sys.path.append('..') # access to drone source without installation
from ardrone2 import ARDrone2, ManualControlException

def scanVersion0( drone, timeout=10.0 ):
    startTime = drone.time
    while drone.time - startTime < timeout:
        sx, sy, sz, sa = 0.0, 0.0, 0.0, 0.3
        drone.moveXYZA( sx, sy, sz, sa )
    drone.hover(0.1) # stop rotation


def testLesson6( drone ):
    try:
        drone.startVideo( record=True, highResolution=True )
        drone.setVideoChannel( front=True )
        drone.takeoff()
        scanVersion0( drone )
    except ManualControlException, e:
        print "ManualControlException"
    if drone.ctrlState == 3: # CTRL_FLYING=3 ... i.e. stop the current motion
        drone.hover(0.1)
    drone.land()
    drone.stopVideo()
    print "Battery", drone.battery



if __name__ == "__main__":
    import launcher
    launcher.launch( sys.argv, ARDrone2, testLesson6 )


# vim: expandtab sw=4 ts=4

