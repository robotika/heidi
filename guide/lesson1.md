Flying autonomous Parrot AR Drone 2 with Python

Lesson 1 - Before you takeoff

In this revised lesson we will test communication with your drone via basic functions provided by Parrot ARDrone2
Wi-Fi API. Power on the AR Drone 2 and connect to it. The drone acts as Access
Point with DHCP - your notebook should get IP like 192.168.1.2.

In this first lesson we will not fly. For all further lessons make
sure that you have enough space available, there is no wind etc. Also always
fly with indoor protective cover even if you fly autonomously outside.

Code:

    import sys
    sys.path.append('..') # access to drone source without installation

To simplify "installation" we added these two lines to all lessons. Then you do
not need to copy any files or set PYTHONPATH. All you need is _git checkout_
and test lessons one by one in _guide directory_.

    from ardrone2 import ARDrone2

This line will import class "ARDrone2". 

    drone = ARDrone2()

Here is hidden all creation work. "drone" is now instance of class ARDrone2,
with initialized connection, starting log files, setting default values for all
internal variables.

So what we can do if flying is too dangerous for the very first lesson? We can wait ;-)

    drone.wait(10.0)      # active waiting for 10 seconds

This is _active waiting_ so you talk to your drone (200 commands per second)
and you collect current drone status. Here are two example what you can be
interested in ... battery and coordinates. 

    print drone.battery
    print drone.coord     # X, Y, Z coordinate

The battery status is reported as percentage. It is useful to print it both
before and after flight. If it is too low (15%?) the drone will not take off
any more and you can quickly see why.

The "drone.coord" are in metres relative to start which is (0,0,0). I have not
test that but I would be curious what will happen if you move the drone
withing 10 seconds of waiting time. Will the coordinates change?

Finally try to press any key during the test of lesson 1. The program should
terminate and write that you did not handle _ManualControlException_ ... do not
worry, we will fix it in the next lesson.

