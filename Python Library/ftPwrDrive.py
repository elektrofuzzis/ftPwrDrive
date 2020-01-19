import ftrobopy
import struct


class ftPwrDrive:

    def __init__(self, dev, txt):
        # definitions
        self.__CMD_SETWATCHDOG = 0  # void setWatchdog( long interval )                                  set watchdog timer

        self.__CMD_SETMICROSTEPMODE = 1  # void setMicrostepMode( mode )                                      set microstep mode - FILLSTEP, HALFSTEP, QUARTERSTEP, EIGTHSTEP, SIXTEENTHSTEP
        self.__CMD_GETMICROSTEPMODE = 2  # uint8_t getMicroStepMode( void )                                   get microstep mode - FILLSTEP, HALFSTEP, QUARTERSTEP, EIGTHSTEP, SIXTEENTHSTEP

        self.__CMD_SETRELDISTANCE = 3  # void setRelDistance( uint8_t motor, long distance )                set a distance to go, relative to actual position
        self.__CMD_SETABSDISTANCE = 5  # void setAbsDistance( uint8_t motor, long distance )                set a absolute distance to go
        self.__CMD_GETSTEPSTOGO = 7  # long getStepsToGo( uint8_t motor )                                 number of needed steps to go to distance

        self.__CMD_SETMAXSPEED = 8  # void setMaxSpeed( uint8_t axis, long maxSpeed )                    set max speed
        self.__CMD_GETMAXSPEED = 9  # long getMaxSpeed( uint8_t axis )                                  get max speed

        self.__CMD_STARTMOVING = 10  # void startMoving( uint8_t motor, boolean disableOnStop )           start motor moving, disableOnStop disables the motor driver at the end of the movement
        self.__CMD_STARTMOVINGALL = 11  # void startMovingAll( uint8_t maskMotor, uint8_t maskDisableOnStop )   same as StartMoving, but using uint8_t masks
        self.__CMD_ISMOVING = 12  # boolean isMoving( uint8_t motor )                                  check, if a motor is moving
        self.__CMD_ISMOVINGALL = 13  # uint8_t isMovingAll(  )                                            return value is uint8_tmask, flag 1 is motor#1, flag2 is motor #2, ...

        self.__CMD_GETSTATE = 14  # uint8_t getState( uint8_t motor )                                  8754321  - flag 1 motor is running, flag 2 endstop, flag 3 EMS, flag 4 HOMING

        self.__CMD_SETPOSITION = 15  # void setPosition( uint8_t motor, long position )                   set position
        self.__CMD_SETPOSITIONALL = 16  # void setPositionAll( long p1, long p2, long p3, long p4 )          set position of all motors
        self.__CMD_GETPOSITION = 17  # long getPosition( uint8_t motor )                                  get position
        self.__CMD_GETPOSITIONALL = 18  # (long,long,long,long) getPositionAll( void )                       get position of all motors

        self.__CMD_SETACCELERATION = 19  # void setAcceleration( uint8_t motor, long acceleration )           set acceleration
        self.__CMD_GETACCELERATION = 20  # long getAcceleration( uint8_t motor )                              get acceleration
        self.__CMD_SETACCELERATIONALL = 21  # void setAccelerationAll( long acc1, long acc2, long acc3, long acc4 ) set acceleration of all motors
        self.__CMD_GETACCELERATIONALL = 22  # (long,long,long,long) getAccelerationAll( void )                   get acceleration of all motors

        self.__CMD_SETSERVO = 23  # void setServo( uint8_t servo, long position )                      set servo position
        self.__CMD_GETSERVO = 24  # long getServo( uint8_t servo )                                     get servo position
        self.__CMD_SETSERVOALL = 25  # void setServoAll( long p1, long p2, long p3, long p4 )             set all servos positions
        self.__CMD_GETSERVOALL = 26  # (long, long, long, long) getServoAll( void )                       get all servo positions
        self.__CMD_SETSERVOOFFSET = 27  # void setServoOffset( uint8_t servo, long Offset )                  set servo offset
        self.__CMD_GETSERVOOFFSET = 28  # long getServoOffset( void )                                        get servo offset
        self.__CMD_SETSERVOOFFSETALL = 29  # void serServoOffsetAll( long o1, long o2, longnt o3, long o4 )     set servo offset all
        self.__CMD_GETSERVOOFFSETALL = 30  # (long, long, long, long) getServoOffsetAll( void )                 get all servo offset
        self.__CMD_SETSERVOONOFF = 31  # void setServoOnOff( uint8_t servo, boolean OnOff )                 set servo pin On or Off without PWM

        self.__CMD_HOMING = 32  # void homing( uint8_t motor, long maxDistance, boolean disableOnStop) homing of motor "motor": run max. maxDistance or until endstop is reached.

        self.__txt = txt
        self.__dev = dev

        # externe Konstanten
        # microstep modes
        self.FULLSTEP = 0
        self.HALFSTEP = 4
        self.QUARTERSTEP = 2
        self.EIGTHSTEP = 6
        self.SIXTEENTHSTEP = 7

        # servo numbers
        self.S1 = 0
        self.S2 = 1
        self.S3 = 2
        self.S4 = 3

        # number of servos
        self.SERVOS = 4

        # enumeration of servo numbers
        self.S = [self.S1, self.S2, self.S3, self.S4]

        # motor numbers
        self.M1 = 1
        self.M2 = 2
        self.M3 = 4
        self.M4 = 8

        # number of motors
        self.MOTORS = 4

        # enumeration of motor numbers
        self.M = [self.M1, self.M2, self.M3, self.M4]

        # flags, i.e. used in getState
        self.ISMOVING = 1
        self.ENDSTOP = 2
        self.EMERCENCYSTOP = 4
        self.HOMING = 8

        # gears
        self.Z10 = 10
        self.Z12 = 12
        self.Z15 = 15
        self.Z20 = 20
        self.Z30 = 30
        self.Z40 = 40
        self.Z58 = 58
        self.WORMSCREW = 5

    def test(self, mode):
        self.__txt.i2c_write_buffer(self.__dev, struct.pack('>BB', self.__CMD_SETMICROSTEPMODE, mode), 2)

    def Watchdog(self, wtime):
        # set wartchog timer
        self.__txt.i2c_write_buffer(self.__dev, struct.pack("<BI", self.__CMD_SETWATCHDOG, wtime), 5)

    def setMicrostepMode(self, mode):
        # set microstep mode - FILLSTEP, HALFSTEP, QUARTERSTEP, EIGTHSTEP, SIXTEENTHSTEP
        self.__txt.i2c_write_buffer(self.__dev, struct.pack("<BB", self.__CMD_SETMICROSTEPMODE, mode), 2)

    def getMicroStepMode(self):
        return self.__txt.i2c_read_buffer(self.__dev, struct.pack("<B", self.__CMD_GETMICROSTEPMODE), 1, 1)

    def setRelDistance(self, motor, distance):
        # set a distance to go, relative to actual position
        buf = struct.pack("<BBI", self.__CMD_SETRELDISTANCE, motor, distance)
        self.__txt.i2c_write_buffer(self.__dev, buf, 6)

    def setAbsDistance(self, motor, distance):
        # set a absolute distance to go
        self.__txt.i2c_write_buffer(self.__dev, struct.pack("<BBI", self.__CMD_SETABSDISTANCE, motor, distance),
                                    6)

    def setAbsDistanceAll(self, d1, d2, d3, d4):
        # set a absolute distance to go for all motors
        self.setAbsDistance(self.M1, d1)
        self.setAbsDistance(self.M2, d2)
        self.setAbsDistance(self.M3, d3)
        self.setAbsDistance(self.M4, d4)

    def getStepsToGo(self, motor):
        # long getStepsToGo( uint8_t motor )
        rec = self.__txt.i2c_read_buffer(self.__dev, struct.pack("<BB", self.__CMD_SETABSDISTANCE, motor), 2, 4)
        return struct.unpack('<I', rec)

    def setMaxSpeed(self, motor, speed):
        # set max speed
        self.__txt.i2c_write_buffer(self.__dev, struct.pack("<BBI", self.__CMD_SETMAXSPEED, motor, speed), 6)

    def getMaxSpeed(self, motor):
        # long getMaxSpeed( uint8_t axis )
        return self.__txt.i2c_read_buffer(self.__dev, struct.pack("<BB", self.__CMD_GETMAXSPEED, motor), 2, 4)

    def startMoving(self, motor, disableOnStop=True):
        # start motor moving, disableOnStop disables the motor driver at the end of the movement
        self.__txt.i2c_write_buffer(self.__dev, struct.pack("<BBB", self.__CMD_STARTMOVING, motor, disableOnStop),
                                    3)

    def startMovingAll(self, maskMotor, maskDisableOnStop=0x0f):  # disable all on stop
        # same as StartMoving, but using masks
        self.__txt.i2c_write_buffer(self.__dev,
                                    struct.pack("<BBI", self.__CMD_STARTMOVINGALL, maskMotor, maskDisableOnStop), 3)
    def isMoving(self, motor):
        return self.__txt.i2c_read_buffer(self.__dev, struct.pack("<BB", self.__CMD_ISMOVING, motor), 2, 4)

    def getState(self, motor):
        return self.__txt.i2c_read_buffer(self.__dev, struct.pack("<BB", self.__CMD_GETSTATE, motor))

    def isMovingAll(self):
        return self.__txt.i2c_read_buffer(self.__dev, struct.pack("<B", self.__CMD_ISMOVINGALL), 2, 4)

    def setPosition(self, motor, position):
        # set position
        self.__txt.i2c_write_buffer(self.__dev, struct.pack("<BBI", self.__CMD_SETPOSITION, motor, position), 6)

    def setPositionAll(self, p1, p2, p3, p4):
        # set position of all motors
        self.__txt.i2c_write_buffer(self.__dev, struct.pack("<BIIII", self.__CMD_SETPOSITIONALL, p1, p2, p3, p4), 17)

    def getPosition(self, motor):
        return self.__txt.i2c_read_buffer(self.__dev, struct.pack("<BB", self.__CMD_GETPOSITION, motor), 2, 4)

    def getPositionAll(self):
        return self.getPosition(self.M1), self.getPosition(self.M2), \
               self.getPosition(self.M3), self.getPosition(self.M4)

    def setAcceleration(self, motor, acceleration):
        # set acceleration
        self.__txt.i2c_write_buffer(self.__dev, struct.pack("<BBI", self.__CMD_SETACCELERATION, motor, acceleration), 6)

    def setAccelerationAll(self, a1, a2, a3, a4):
        # set acceleration of all motors
        self.__txt.i2c_write_buffer(self.__dev, struct.pack("<BIIII", self.__CMD_SETACCELERATIONALL, a1, a2, a3, a4),
                                    17)

    def getAcceleration(self, motor):
        return self.__txt.i2c_read_buffer(self.__dev, struct.pack("<BB", self.__CMD_GETACCELERATION, motor), 2, 4)

    def getAccelerationAll(self):
        return self.getAcceleration(self.M1), self.getAcceleration(self.M2), \
               self.getAcceleration(self.M3), self.getAcceleration(self.M4)

    def setServo(self, servo, position):
        # set servo position
        self.__txt.i2c_write_buffer(self.__dev, struct.pack("<BBI", self.__CMD_SETSERVO, servo, position), 6)

    def getServo(self, servo):
        return self.__txt.i2c_read_buffer(self.__dev, struct.pack("<BB", self.__CMD_GETSERVO, servo), 2, 4)

    def setServoAll(self, p1, p2, p3, p4):
        # set all servos positions
        self.__txt.i2c_write_buffer(self.__dev, struct.pack("<BIII", self.__CMD_SETSERVOALL, p1, p2, p3, p4), 17)

    def getServoAll(self):
        return self.getServo(self.S1), self.getServo(self.S2), self.getServo(self.S3), self.getServo(self.S4)

    def setServoOffset(self, servo, offset):
        # set servo offset
        self.__txt.i2c_write_buffer(self.__dev, struct.pack("<BBI", self.__CMD_SETSERVOOFFSET, servo, offset), 6)

    def getServoOffset(self, servo):
        return self.__txt.i2c_read_buffer(self.__dev, struct.pack("<BB", self.__CMD_GETSERVOOFFSET, servo), 2, 4)

    def setServoOffsetAll(self, o1, o2, o3, o4):
        # set servo offset all
        self.__txt.i2c_write_buffer(self.__dev, struct.pack("<BIII", self.__CMD_SETSERVOOFFSETALL, o1, o2, o3, o4), 17)

    def getServoOffsetAll(self):
        return self.getServoOffset(self.S1), self.getServoOffset(self.S2),\
               self.getServoOffset(self.S3), self.getServoOffset(self.S4)

    def setServoOnOff(self, servo, on):
        # set servo pin On or Off without PWM
        self.__txt.i2c_write_buffer(self.__dev, struct.pack("<BBB", self.__CMD_SETSERVOONOFF, servo, on), 3)

    def homing(self, motor, maxDistance, disableOnStop=True):
        # homing of motor using end stop
        self.__txt.i2c_write_buffer(self.__dev,
                                    struct.pack("<BBIB", self.__CMD_HOMING, motor, maxDistance, disableOnStop),
                                    7)
