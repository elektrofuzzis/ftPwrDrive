# ftPwrDrive
The ftPwrDrive is a i²c based stepper motor driver for fischertechnik, Arduino &amp; Raspberry. The project contains the stepper motor drivers hardware, the firmware and interface libraries to use the hardware with multiple plattforms.

Please checkout the <a href="https://github.com/elektrofuzzis/ftPwrDrive/wiki">online manual</a>.
Since the hardware is a complex smd pcb, you could buy a plug&play device at <a href="https://gundermann-software.de/shop/">gundermann.org</a>.


<hr width="100%" />

Schrittmotortreiber für fischertechnik, Arduino &amp; Raspberry
Bitte lesen Sie das <a href="https://github.com/elektrofuzzis/ftPwrDrive/wiki">Online Manual</a>.


#Arduino & ftDuino Interface

- Download "ftDuino Library/ftPwrDrive.zip"
- Start Arduino IDE 
- Use Sketch/Include Library/Add .zipFile to import the interface definitions in ftPwrDrive.zip

Afterwards you could import ftPwrDrive.h

#Version 0.93

ftPwrDrive libary (ftDuino, Arduino)
- setAbsDistanceAll: set absolute distances for all motors
- wait: wait until all motors completed their work
- homing: homing of a motor using end stop
- isHoming: check, if homing is active 
- getState: additional flag #4 homing
- Some minor stability fixes

Firmware
- homing-Functions  
