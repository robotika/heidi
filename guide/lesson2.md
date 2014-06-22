Flying autonomous Parrot AR Drone 2 with Python

Lesson 2 - Manual emergency stop

Example in Lesson 1 was the simplest possible, but let's face it, it was also
a dangerous one. Why? What if anything goes wrong? What will you do? What drone
will do?

In this lesson we will introduce "ManualControlException". As soon as you press
any key the normal code execution will switch to "manual" and drone will land.
Otherwise the code is very similar (the hovering time is set to 10 seconds, so
you can interrupt it sooner with hitting the keyboard).

Note about differences in used operating systems. In Windows you can use
"msvcrt.kbhit" which is not available in Linux. Workaround is usage of extra
library (PyGame), which provides simple interface for keyboard scanning.

