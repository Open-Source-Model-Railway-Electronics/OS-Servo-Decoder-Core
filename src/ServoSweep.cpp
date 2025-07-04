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

#include "ServoSweep.h"
#include "macros.h"
#include <EEPROM.h>

// use constructor 1 if you have no optional relay
// use the other if you 

const int STORE_POSITIONS   = 0b10000000 ;
const int DEFAULT_BITS      = 0b00111110 ;

const int SERVO_EE_SIZE = 4 ;
/**
 * @brief Construct a new Servo Sweep:: Servo Sweep object
 * 
 * @param _servoPin pin number of servo
 * @param _min      default position for the 0 state
 * @param _max      default position for the 1 state
 * @param _speed    time in ms between degrees
 * @param _turnOff  turn of the servo when in position
 */
ServoSweep::ServoSweep()                    // constructor 1
{
    // servoPin = _servoPin ;
    // servoSpeed = _speed ;
    // servoMin = _min ;
    // servoMax = _max ;
   
    // updateMiddle() ;

    // relayPin = 0xFF ; // no relay

    // if( _turnOff ) turnOff = 1 ;
    // else           turnOff = 0 ;
}

/**
 * @brief Construct a new Servo Sweep:: Servo Sweep object
 * 
 * @param _servoPin pin number of servo
 * @param _min      default position for the 0 state
 * @param _max      default position for the 1 state
 * @param _speed    time in ms between degrees
 * @param _turnOff  turn of the servo when in position
 * @param _relayPin pin number of relay. Switches when servo is halfway
 */
// ServoSweep::ServoSweep(/* uint8_t _servoPin, uint8_t _min, uint8_t _max, uint8_t _speed, uint8_t _turnOff, uint8_t _relayPin*/ )      // constructor 2
// {    
    // servoPin = _servoPin ;
    // servoSpeed = _speed ;
    // servoMin = _min ;
    // servoMax = _max ;

    // updateMiddle() ;

    // relayPin = _relayPin ;

    // if( _turnOff ) turnOff = 1 ;
    // else           turnOff = 0 ;
// }

void ServoSweep::begin( uint8_t _servoPin, uint8_t _min, uint8_t _max, uint8_t _speed, uint8_t _relayPin )
{
    turnOff = 1 ;

    if( eeAddress != 0xFFFF ) // If EEPROM present
    {
        uint8_t flags = EEPROM.read(eeAddress+2) ;
        // printNumberln("flags ",flags);

        if( (flags & DEFAULT_BITS) > 0 ) // if any of these bit are set, we must initialize the EEPROM
        {
            EEPROM.write( eeAddress+0,   servoMin ) ;
            EEPROM.write( eeAddress+1,   servoMax ) ;
            EEPROM.write( eeAddress+2, servoSpeed ) ;
            EEPROM.write( eeAddress+3,          0 ) ; // last state

            // Serial.println("SETTING DEFAULTs");
            // printNumber_("servoMin ",servoMin);
            // printNumber_("servoMax ",servoMax);
            // printNumberln("last state ",0);
            
        }

        servoMin   = EEPROM.read( eeAddress+0 ) ;
        servoMax   = EEPROM.read( eeAddress+1 ) ;
        servoSpeed = EEPROM.read( eeAddress+2 ) ;
        updateMiddle() ;

        uint8 status = EEPROM.read(eeAddress+3) ;
        if( eeFlags & STORE_POSITIONS )
        {
            if( status & 1 ) { pos = servoMax ; state = 1 ; }
            else             { pos = servoMin ; state = 0 ; }
        }

        if( status & (1<<6) ) { invertRelay = 1 ; }
        else                  { invertRelay = 0 ; }
        // Serial.println("beginning") ;
        // printNumber_("servoMin ",servoMin);
        // printNumberln("servoMax ",servoMax);
        // printNumberln("LOADING state: ", state) ;
    }

    if( relayPin != 0xFF ) pinMode( relayPin, OUTPUT ) ;

    startUp = 1 ;
}

void ServoSweep::setState( uint8_t _state )
{
    state = _state ;

    if( eeFlags & STORE_POSITIONS )
    {
        uint8 newState = EEPROM.read( eeAddress+2 ) ;
        bitWrite( newState, 0, state ) ;
        EEPROM.write( eeAddress+2, newState ) ;
    }
}

void ServoSweep::setSpeed( uint8_t _speed )
{
    servoSpeed = _speed ;
}

uint8_t ServoSweep::getState()
{
    return state ;
}

void ServoSweep::updateMiddle()
{
    middlePosition = ( (long)servoMax - (long)servoMin ) / (long)2 + (long)servoMin ;
}

void ServoSweep::setMin( uint8_t _min)
{
    servoMin = _min ;
    EEPROM.write( eeAddress+0, servoMin ) ;
    updateMiddle() ;
}

void ServoSweep::setMax( uint8_t _max)
{
    servoMax = _max ;
    EEPROM.write( eeAddress+1, servoMax ) ;
    updateMiddle() ;
}

void ServoSweep::manualOverride( uint8_t pos)
{
    servoSetpoint = pos ;
    override = 1 ;
    updateTime = millis() ;
}

void ServoSweep::manualRelease()
{
    override = 0 ;
}

void ServoSweep::toggleRelay()
{
    uint8 status = EEPROM.read( eeAddress+2 ) ;
    status ^= (1<<6) ;
    EEPROM.update( eeAddress+2, status ) ;

    invertRelay ^= 1 ;
}

void ServoSweep::increment()
{
    if( state && servoMax <= 160 ) { servoMax += 2 ; }
    else if(     servoMin <= 160 ) { servoMin += 2 ; }
    updateMiddle() ;
}

void ServoSweep::decrement()
{
    if( state && servoMax >=   20 ) { servoMax -= 2 ; }
    else if(     servoMin >=   20 ) { servoMin -= 2 ; }
    updateMiddle() ;
}

void ServoSweep::commitSettings()
{
    EEPROM.update( eeAddress+0,   servoMin ) ;
    EEPROM.update( eeAddress+1,   servoMax ) ;
    EEPROM.update( eeAddress+2, servoSpeed ) ;
}


uint8_t ServoSweep::sweep ( )
{
    if( millis() - lastTime > servoSpeed )
    {       lastTime = millis() ;


        if( relayPin != 0xFF )
        {
            // first operand checks if relay must be on or off, 2nd operand checks if min is smaller than max and comensates with XOR. 3 operand is manual inverting of relay
            uint8_t relayState = 
                (pos < middlePosition ? 1 : 0) 
                                ^ 
                            invertRelay ;

            digitalWrite( relayPin, relayState ) ;
        }
     
        uint8_t setPoint ;

        if( state ) setPoint = servoMax ; // get set point
        else        setPoint = servoMin ;

        if( override ) setPoint = servoSetpoint ;

        if( pos < setPoint ) pos ++ ;   // follow positon to setpoint
        if( pos > setPoint ) pos -- ;

        if( prevPos != pos || startUp == 1 ) {          // if position has changed or we have just booted..
            prevPos  = pos ;

            if( servoPin != 255 ) servo.write( pos ) ;
            updateTime = millis() ;

            if(( pos != setPoint || startUp == 1 ) 
            &&  servo.attached() == false ) // attach motor if needed
            {
                // Serial.println("engaging servo");
                servo.attach( servoPin ) ;
                startUp  = 0 ;
            }
            
            //return pos ;
        }
    }

    if( servo.attached() 
    &&  turnOff == 1 
    && ( millis() - updateTime > 500) )
    {
        servo.detach( ) ; // detach motor if needed
    }
}

void ServoSweep::setEeAddress( uint16_t _eeAddress )
{
    static uint16 firstAddress = 0xFFFF ; // delibarately used static to automatically increase addresses

    if( firstAddress == 0xFFFF )
    {
        firstAddress = _eeAddress ;
    }

    eeAddress     =  firstAddress ;      // set my own eeAddress
    firstAddress += SERVO_EE_SIZE ; // increment for next servoSweep object

    eeFlags = STORE_POSITIONS ;
}


void ServoSweep::useEEPROM( uint16_t _eeAddress ) // this one is needed for the very first servo object.
{
    setEeAddress( _eeAddress ) ;
}

void ServoSweep::useEEPROM( )                   // use this one for all the others
{
    setEeAddress( 0x0000 ) ;
}

/* Flag that this servo must use default values */
void ServoSweep::reset()                        
{
    EEPROM.write( eeAddress+2, DEFAULT_BITS ) ; // flag begin method to load defaults
    //begin() ;                                   // call begin again to load the defaults
}