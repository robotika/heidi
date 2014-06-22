Flying autonomous Parrot AR Drone 2 with Python

Lesson 1

In this lesson we will demonstrate basic functions provided by Parrot ARDrone2
Wi-Fi API. Power on the AR Drone 2 and connect to it. The drone acts as Access
Point with DHCP - your notebook should get IP like 192.168.1.2.

Note, that this first simple program does not have _any_ (!) protection so make
sure that you have enough space available, there is no wind etc. Also always
fly with indoor protective cover even if you fly autonomously outside.

Code:

from ardrone2 import ARDrone2

This line will import class "ARDrone2". In order to successfully import the
class it is necessary to set PYTHONPATH to directory where you can find
"ardrone2.py". An alternative is to install drone files into Python
directories.


drone = ARDrone2()

Here is hidden all creation work. "drone" is now instance of class ARDrone2,
with initialized connection, starting log files, setting default values for all
internal variables.

drone.takeoff()

Parrot provides automatic takeoff (you can have a look in the "ardrone2.py"
source code and see that it is shortcut for repeated AT command "AT*REF=%i,
0b10001010101000000001000000000") until drone status changes to state
"flying".

drone.hover( 3.0 )

Hovering is state, when the drone is keeping the same coordinate. This is
mostly safe function except moments when the drone pushes against some obstacle.
The it does not have a space how to compensate the tilt and you can easily
brake it (as I did my first drone when I was testing in a very small room).
Make sure you have enough space or rather jump to Lesson 2, where you have
some kind of "emergency stop".

The parameter for hovering is time in seconds (we try to keep metric units if
possible, i.e. seconds, meters and angles in radians).

drone.land()

The last step should be always to land. The command is similar to take off
("AT*REF") except it sets different flags and waits for changing state to
"landed".

