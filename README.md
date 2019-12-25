# ftPwrDrive
The ftPwrDrive is a i²c based stepper motor driver for fischertechnik, Arduino &amp; Raspberry. The project contains the stepper motor drivers hardware, the firmware and interface libraries to use the hardware with multiple plattforms.

Please checkout the <a href="https://github.com/elektrofuzzis/ftPwrDrive/wiki">online manual</a>. Since the hardware is a complex smd pcb, you could buy a plug&play device at <a href="https://gundermann-software.de/shop/">gundermann.org</a>.

To check out the latest releases please follow up the version history.

<hr width="100%" />

Der ftPwrDrive ist ein Schrittmotortreiber für fischertechnik, Arduino &amp; Raspberry auf i²C-Basis. Das Projekt beinhaltet sowohl doe Schrittmotortreiberhardware und die notwendige Firmware als auch Interface Bibliotheken für unterschiedliche Plattformen.

Bitte lesen Sie das <a href="https://github.com/elektrofuzzis/ftPwrDrive/wiki">Online Manual</a>. Die Hardware besteht aus einer komplexen, SMD-bestückten Platine. Sie können den ftPwrDrive auch als fertiges Gerät bei <a href="https://gundermann-software.de/shop/">gundermann.org</a> bestellen.

Das Changelog für die aktuelle Version finden sie hier.


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
