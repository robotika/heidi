"""
  Lesson 5 - fly at given height
"""
import sys
sys.path.append('..') # access to drone source without installation
from ardrone2 import ARDrone2, ManualControlException

def testLesson5( drone ):
    desiredSpeed = 0.8 # in meters per second
    desiredHeight = 1.5
    try:
        drone.takeoff()
        startTime = drone.time
        while drone.time - startTime < 10.0:
     
            altitude = desiredHeight
            if drone.altitudeData != None:
                altVision = drone.altitudeData[0]/1000.0
                altSonar = drone.altitudeData[3]/1000.0
                altitude = (altSonar+altVision)/2.0
                if abs(altSonar-altVision) > 0.5:
                    print altSonar, altVision
                    altitude = max( altSonar, altVision ) # sonar is 0.0 sometimes (no ECHO)

            sz = max( -0.2, min( 0.2, desiredHeight - altitude ))
            if altitude > 2.5:
                # wind and "out of control"
                sz = max( -0.5, min( 0.5, desiredHeight - altitude ))
            sx = max( 0, min( drone.speed, desiredSpeed - drone.vx ))
            sy, sa = 0.0, 0.0
            drone.moveXYZA( sx, sy, sz, sa )

    except ManualControlException, e:
        print "ManualControlException"
    if drone.ctrlState == 3: # CTRL_FLYING=3 ... i.e. stop the current motion
        drone.hover(0.1)
    drone.land()
    print "Battery", drone.battery



if __name__ == "__main__":
    import launcher
    launcher.launch( sys.argv, ARDrone2, testLesson5 )


# vim: expandtab sw=4 ts=4

