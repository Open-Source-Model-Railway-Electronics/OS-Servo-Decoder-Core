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

#include "ST.h"


/*   TIMERS   */

BaseTimer::BaseTimer()
{
    Q = 0 ;
    PT = 0 ;
    ET = 0 ;
}


uint8_t TIMER_ON::update(uint8_t IN) 
{
    Q = 0 ;
    if( !IN )
    {
        ET = 0 ;
        startTrigger = 1 ;
        endTrigger   = 1 ;
    }
    else if( startTrigger )
    {
        startTrigger = 0 ;
        startTime = millis() ;
    }

    if( IN )
    {
        ET = millis() - startTime ;
    }

    if( IN && ET >= PT && endTrigger == 1 )
    {
        endTrigger = 0 ;
        Q = 1 ;
    }

    return Q ;
}


uint8_t TIMER_OFF::update(uint8_t IN) 
{
    Q = 0 ;
    if( IN )
    {
        ET = 0 ;
        startTrigger = 1 ;
        endTrigger   = 1 ;
    }
    else if( startTrigger )
    {
        startTrigger = 0 ;
        startTime = millis() ;
    }

    if( !IN )
    {
        ET = millis() - startTime ;
    }

    if( !IN && ET >= PT && endTrigger == 1 )
    {
        endTrigger = 0 ;
        Q = 1 ;
    }

    return Q ;
}


uint8_t TIMER_BLEEP::update(uint8_t IN) 
{
    if( !IN )
    {
        startTime = millis() ;
        ET = 0 ;
    }

    if( IN )
    {
        ET = millis() - startTime ;
        if( ET >= PT )
        {
            startTime = millis() ;
            ET = 0 ;
            Q = 1 ;
        }
        else
        {
            Q = 0 ;
        }
    }
    else
    {
        Q = 0 ;
    }

    return Q ;
}


uint8_t TIMER_PULSE::update(uint8_t IN) 
{
    if( !IN )
    {
        startTime = millis() ;
        endTrigger = 1 ;
        ET = 0 ;
    }

    if( endTrigger == 1 )
    {
        ET = millis() - startTime ;
        if( ET >= PT )
        {
            endTrigger = 0 ;
        }
    }
    else
    {
        ET = 0 ;
    }

    if( IN && endTrigger )
    {
        Q = 1 ;
    }
    else
    {
        Q = 0 ;
    }

    return Q ;
}


/*   TRIGGERS   */
R_TRIG::R_TRIG()
{
}

void R_TRIG::update( uint8_t IN )
{
    Q = 0 ;

    if( IN != EN  )
    {
        Q   = IN ;
        EN  = IN ;
    }
} 

F_TRIG::F_TRIG()
{
}

void F_TRIG::update( uint8_t IN )
{
    Q = 0 ;

    if( IN != EN )
    {
        Q   = IN ^ 1 ;
        EN  = IN ;
    }
}

C_TRIG ::C_TRIG ()
{
    Q   = 0 ;
    EN  = 0 ;
}

void C_TRIG ::update( uint8_t IN )
{
    Q = 0 ;
    if( IN != EN  )
    {
        Q = 1 ;
        EN  = IN ;
    }
}

T_TRIG ::T_TRIG ()
{
    Q   = 0 ;
    EN  = 0 ;
}

void T_TRIG ::update( uint8_t IN )
{
    if( IN == 1 && EN == 0 )
    {
        Q ^= 1 ;
    }
    EN  = IN ;
}


/***** FLIP FLOPS *****/
SR::SR()
{
    Q = 0 ;
}

void SR::update( uint8_t S, uint8_t R )
{
    if( R )       Q = 0 ;
    else if( S )  Q = 1 ;
}


RS::RS()
{
    Q = 0 ;
}

void RS::update( uint8_t S, uint8_t R )
{
    if( S )       Q = 1 ;
    else if( R )  Q = 0 ;
}

MEM::MEM()
{
}

void MEM::update( uint8_t IN )
{
    if( IN ) Q = 1 ;
    else     Q = 0 ;
}

/***** COUNTER ****/
BaseCounter::BaseCounter() {}

uint8_t UP_COUNTER::count( uint8_t IN, uint16_t target )
{
    Q = 0 ;

    if( !IN )
    {
        counter = 0 ;
    }

    else
    {
        if( counter  < target ) counter ++ ;
        if( counter == target ) Q = 1 ;
    }

    return Q ;
}


uint8_t DOWN_COUNTER::count( uint8_t IN, uint16_t startPoint )
{
    Q = 0 ;

    if( !IN )
    {
        counter = startPoint ;
    }

    else
    {
        if( counter  > startPoint ) counter -- ;
        if( counter ==          0 ) Q = 1 ;
    }
    
    return Q ;
}

/******* RAMP GENERATOR *******/
RAMP_GEN::RAMP_GEN()
{
    setpoint  = 0 ;
    EN        = 0 ;
    Y         = 0 ;
    interval  = 0 ;
    Q         = 0 ;
    lastStep  = millis() ;
}

void RAMP_GEN::update()
{
    if( !EN ) return ;

    uint32_t now = millis();
    if( now - lastStep < interval ) return ;

    lastStep = now ;

    if( Y < setpoint  ) Y++ ;
    if( Y > setpoint  ) Y-- ;

    Q = ( Y == setpoint  ) ;
}
