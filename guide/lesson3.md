Flying autonomous Parrot AR Drone 2 with Python

Lesson 3 - Logging

Every flight is unique and you will never be able to exactly repeat it. How can
you then develop anything or debug what just happened? The key part is log
files - all commands and input data are logged so you can replay them any time
later. Note that the program has to be deterministic and not dependent on other
source (like system time - otherwise you would have to log it as source too).

In this example we will replay existing logs. Note, that log files were
generated automatically also in previous lessons, but we were not able to
replay them yet.

In this lesson we changed main code into function "testLesson3" and use
"launcher" to parse parameters and call the main function.

How to replay the flight? Or maybe even what are the command line parameters
now? It is name of the program and reserved parameter for the main code (note,
task number or name of configuration file, for example). 3rd parameter is then
name of the "metalog file". What is that? It is file from "logs" directory
starting with keyword "meta". There are several sources involved and if you
want to be able to repeat them with identical results you have to for example
also record the moment when you hit the emergency stop button/keyboard. Metalog
then contains references to names of all other files.

== Task 1

Q: What was your battery level _before_ takeoff?

A: Because print is not changing your drone behavior, you can add there:
  print "Battery", drone.battery
Because all navdata (navigation data from the drone) are logged you can add the print(s) afterwards!



== Task 2

Q: What was the maximum height during your very first flight?

A: Well, this will be a little bit tricky. We are logging everything even if
you do not ask for it. It is something like "black box" on the airplane ... you
cannot put it there after the crash. It also fulfills similar mission - if
something goes wrong you can analyse it and prevent it from repetition.

So back to height from your first flight. There are several sources about drone
altitude (we will discuss/test it in some later lesson) and they are all merged
into Z-coordinate available as 3rd variable in drone.coord tuple. You can add
"print drone.coord[2]" just before you call drone.land().

Is there a problem replaying the metalog file? Actually there should be ;-).
Why? Because in your first example you hover for 3 seconds while in your lesson
3 it was set to 10 seconds. You should see assert like:

  File "m:\git\heidi\ardrone2.py", line 147, in update
    assert fileCmd==cmd, "Diff from file: %s, cmd %s" % (fileCmd, cmd)
AssertionError: Diff from file: AT*CONFIG_IDS=, cmd AT*FTRIM=237,

This means that your code/program generates different commands then during the
flight. You can experiment and change 10 to 3 to see if this assert goes away
(it should). Or you add extra parameter "F" (as "False" for assert checking)
and will ignore new commands and only process incoming sensor data.

