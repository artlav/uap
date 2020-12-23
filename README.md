# Orbiter Universal Autopilots

Orbiter Universal Autopilots is a framework for writing and running universal and sequence-able autopilots in Orbiter Space Flight Simulator( http://orbit.medphys.ucl.ac.uk/ ).
If you heard about Redshift by Bernd R. Fix., UAP is something similar, but much better.

## What can it be used for?

From the player point of view, it's an MFD that one can use to define a series of actions to be performed on the current vessel, like lift off, climb to orbit, wait for apoapsis, circularize.  
 - The sequence of autopilots run fully automatic, starting next one as the previous completes.  
 - It is designed to be saved to scenario at any point, and be resumed correctly on reload.  
 - The framework allows autopilots to be orientation � independent, thus, ascent autopilot can be run with any engine, for example.  
  
From a developer point of view, it could have been a nice framework for writing autopilots, but unless there is a noticeable interest in it, no SDK release is planned.  
  
Dump of code as of alpha v0.3.1 (110613)  
  
Full add-on files are at: https://www.orbithangar.com/showAddon.php?id=06ea3517-c6d9-4119-a198-aecea8c3e233  
  
It should work, but probably won't easily compile.

