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

