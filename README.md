# ftPwrDrive
I²C based stepper motor driver for fischertechnik, Arduino &amp; Raspberry

Please checkout the <a href="https://github.com/elektrofuzzis/ftPwrDrive/wiki">online manual</a>.

<line x1="0" x2="100%">

I²C

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
