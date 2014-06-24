"""
  Lesson 4 - fly forward
"""
from ardrone2 import ARDrone2, ManualControlException

def testLesson4( drone ):
    try:
        drone.takeoff()
        startTime = drone.time
        while drone.time - startTime < 2.0:
            sx, sy, sz, sa = 0.1, 0.0, 0.0, 0.0
            drone.moveXYZA( sx, sy, sz, sa )
    except ManualControlException, e:
        print "ManualControlException"
    if drone.ctrlState == 3: # CTRL_FLYING=3 ... i.e. stop the current motion
        drone.hover(0.1)
    drone.land()
    print "Battery", drone.battery



if __name__ == "__main__":
    import launcher
    import sys
    launcher.launch( sys.argv, ARDrone2, testLesson4 )


# vim: expandtab sw=4 ts=4

