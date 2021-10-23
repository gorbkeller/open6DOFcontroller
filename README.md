# open6DOFcontroller
## Gordon Keller
### 10/23/21

*** An open-source 6DOF (Degrees Of Freedom) controller. ***

This project leverages several different packages to emulate 6DOF input akin to the closed-source/proprietary hardware 3D Connexion SpaceMouse.

In this original form, it combines:
* FIRMWARE: The QMK Firmware project (link) -- TBD
* ELECTRONICS: KiCAD (link)
* HARDWARE/MODELS: Autodesk Fusion360 (link), though intention to eventually port to FreeCAD.

This controller is a departure from the pressure sensor-based design approach -- we use hall-effect sensors 'round these parts. The algorithms I (Gordon "gorbkeller" Keller) developed to convert magnetic signals to 6DOF aren't very sophistocated, so get to forkin' or contributin' signal processing engineers!
Theoretically, it can be used to control any 3D virtualized perspective/object/whathaveyou. Eventually, I want to use it to fly the Lynchpin drone (link) in simulation and in IRL too ;)
I've included in this first installment a driver written for Fusion360 camera view control as an example useage. This is written using the Fusion360 Scripting API, though I have a hunch that there's a better way to do it.

The outputs of the one-handed module are given in either XYZ-translation + WIJK-Quaternion ("pose-mode") or XYZ-translation + Roll/Pitch/Yaw ("euler-mode"). 

Some people might not like the 6DOF with one hand control, so a fork of this repo (3DOF-3DOF) has been created which splits the device in two for use with both hands. This method involves less signal processing but requires multiple PCBs.

In-depth plan TBD! - gribgrob_k

