/*
 * Copyright (C) 2024 Sebastiaan Knippels, Train-Science
 *
 * To the extent possible under law, the person who associated CC0 with this work
 * has waived all copyright and related or neighboring rights to this work.
 *
 * This work is published from: The Netherlands
 *
 * You can copy, modify, distribute and perform the work, even for commercial purposes,
 * all without asking permission. See the full license for details at:
 * https://creativecommons.org/publicdomain/zero/1.0/
 */

/*
ServoSweep library written by S Knippels
Public Domain
*/

#include <Arduino.h>
#include <Servo.h>

const int NO_POS = 0xFF ;

class ServoSweep
{
public:
    ServoSweep( /*uint8_t _servoPin, uint8_t _min, uint8_t _max, uint8_t _speed, uint8_t _turnOff */ ) ;                    // constructor 1
    //ServoSweep( /*uint8_t _servoPin, uint8_t _min, uint8_t _max, uint8_t _speed, uint8_t _turnOff, uint8_t _relayPin*/ ) ;  // constructor 2
    uint8_t sweep( ) ;
    void setState( uint8_t _state ) ;
    uint8_t getState() ;
    void begin(  uint8_t, uint8_t, uint8_t, uint8_t, uint8_t ) ;
    void setMin( uint8_t _min) ;
    void setMax( uint8_t _max) ;
    void increment( ) ;
    void decrement( ) ;
    void commitSettings( ) ;
    void toggleRelay( ) ;
    void useEEPROM( uint16_t _eeAddress ) ;
    void useEEPROM( ) ;
    void reset( ) ;
    void manualOverride( uint8_t pos ) ;
    void manualRelease() ;
    void setSpeed( uint8_t _speed ) ;

private:
    void        updateMiddle() ;
    void        setEeAddress( uint16_t _eeAddress ) ;
    Servo       servo ;
    uint32_t    lastTime ;
    uint8_t     pos ;
    uint8_t     override : 1;
    uint8_t     tuning   : 1;
    uint8_t     state : 1;
    uint8_t     prevPos ;
    uint8_t     servoPin : 6;
    uint8_t     servoSpeed ;
    uint8_t     servoMin ;
    uint8_t     servoMax  ;
    uint8_t     middlePosition ;
    uint8_t     servoSetpoint ;
    uint8_t     relayPin : 6 ;
    uint8_t     turnOff : 1 ;
    uint8_t     invertRelay : 1 ;
    uint8_t     startUp : 1 ;
    uint8_t     eeFlags ;
    uint16_t    eeAddress = 0xFFFF ;
    uint32_t    updateTime ;
} ;