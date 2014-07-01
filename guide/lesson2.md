Flying autonomous Parrot AR Drone 2 with Python

Lesson 2 - Manual emergency stop and basic functions

We have revised flying in Lesson 1. It was the simplest possible code, but
let's face it, it was also a dangerous one. Why? What if anything goes wrong?
What will you do? What drone will do? Now is the "flying code" moved to this
lesson 2.

First we will introduce "ManualControlException". As soon as you press
any key the normal code execution will switch to "manual" and drone will land.

Note about differences in used operating systems. In Windows you can use
"msvcrt.kbhit" which is not available in Linux. Workaround is usage of extra
library (PyGame), which provides simple interface for keyboard scanning.


Code:

    from ardrone2 import ARDrone2
    drone = ARDrone2()

These bits we already know from the Lesson 1. So we have "drone" ready to
accept commands and you can read current status in internal variables.

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

----------------------

Example of things !!NOT DO DO!!

2014-06-23:
... I just successfully completed lesson 1, against your advice inside my small office ;-).
The drone managed to take off, hover for three seconds just below and somewhat against the ceiling, and then it landed safely.
... 

DO NOT DO THAT! ... you can easily break your new toy! Believe me, I have done that (two years ago)!

