#include "config.h"

#include "src/macros.h"
#include <EEPROM.h>
#include "src/debounceClass2.h"
#include "src/NmraDcc.h"
#include "src/ServoSweep.h"
#include "src/ST.h"

/* TODO  
 // need backward compatability to mk1 version 1
 // improve all of EEPROM
    the serv objects uses 4 bytes each and start at 0x0000. The max is 12 servos so 48 bytes starting from zero are used
    deadbeef used 1020-1023
    settings uses a few bytes after 900. SHould be ok

 // implement all servo decoder version IO, also include mardec,
 // implement Speed changes using F8 and throttle
 // change the constructor so now variables are needed no more.
 // Also change the servoSweepClass so the speed is also committed



*/

enum
{
    idle, // check all conditions? 
    speed_zero,
    configureServo,
} ;

uint8           index ;
uint8           servoSpeed ;
const uint16    servoAddress = 0x50 ;
uint16          myAddress ;
uint8           waiting4address ;
uint8           state = idle ;
NmraDcc         dcc ;

uint8           dccIndex = 0xFF ;
uint8           servoSetpoint  ;

R_TRIG          F0 ;
R_TRIG          F1 ;
R_TRIG          F2 ;
R_TRIG          F3 ;
C_TRIG          F4 ;
C_TRIG          F5 ;
C_TRIG          F6 ;
C_TRIG          F7 ;
R_TRIG          F8 ;

const int       DEADBEEF_EE_ADDRESS    = 1020 ; // 1020, 1021, 1022, 1023
const int       EE_SETTINGS            =  900 ;

const int       LOCO_FUNCTIONS_OFF = 0 ;
const int       FANTASTIC_FOUR     = 1 ;
const int       EIGHT_BALL         = 2 ;

typedef struct
{
    uint16       myAddress : 12 ; 
    uint16 uniqueAddresses :  1 ;
    uint16          dccExt :  1 ; 
    uint16   locoFunctions :  2 ; // enable the usage of locomotive function. this can be 1 address with 16 functions or 4 addresses with 4 functions
    uint16          rcn213 :  1 ; // address offset for multimaus icw old roco boosters. Addresses are to be substracted with 4 when option is OFF.
    // I used double addresses so any system can turn on or off this setting. Address 1001 and 997 both turn it ON
    // addresses 1000 and 996 turn it OFF.
    // the default = ON
} Settings ;

Settings       settings ;
const Settings defaultSettings =
{
    .myAddress       = 1,
    .uniqueAddresses = 0 ,
    .dccExt          = 0,
    .locoFunctions   = LOCO_FUNCTIONS_OFF,
    .rcn213          = 1
};

const int defaultMin   =  80 ;
const int defaultMax   = 100 ;
const int defaultSpeed =  40 ;

// DECODER UNIQUE SERVO SETTINGS
ServoSweep  servo[nServos] = {};
Debouncer   switches[nSwitches] = {} ;

uint8     blinkCounter ;
uint8     blinkMax ;
uint8     flashing ;
uint32    blinkInterval = 500 ;

void blinkLed( uint8 blinks ) 
{
    blinkMax = 2*blinks-1 ;
    blinkCounter = 0 ;
    blinkInterval = 100 ;
    flashing = 1 ;
}

void statusLed()
{
    if( blinkCounter > blinkMax ) flashing = 0 ;

    REPEAT_MS( blinkInterval )
    {
        if( flashing )  blinkCounter ++ ;
        else if( waiting4address )         blinkInterval =  333 ;
        else if( state == speed_zero     ) blinkInterval =  100 ;
        else if( state == configureServo ) blinkInterval = 1000 ;
        else                               blinkInterval =    1 ; // just normal idle mode ;

        if( blinkInterval == 1 ) PORTB &= ~(1 << 5) ;  // just be on, if there is nothing going on
        else                     PORTB ^=   1 << 5  ;  // toggle
    }
    END_REPEAT
}


#if defined SWITCH_LAYOUT_4BTN
void process4ButtonLayout()
{
    uint8 stateDown = switches[DOWN].state ;
    uint8 stateUp   = switches[ UP ].state ;

// ************ FINE TUNING SERVO POSITION ****************
    REPEAT_MS( 200 )
    {
        if( switches[DOWN].state == LOW && switches[UP].state == LOW )
        {
            //servo[index].manualOverride( 90 ) ;
        }
        else if( switches[DOWN].state == LOW )
        {
            waiting4address = 0 ;
            servo[index].decrement() ;
            servo[index].manualRelease() ;
            blinkLed(1) ;
        }
        else if( switches[UP].state   == LOW )
        {
            waiting4address = 0 ;
            servo[index].increment() ; 
            servo[index].manualRelease() ;
            blinkLed(2) ;
        }
    }
    END_REPEAT

    if( stateDown == RISING || stateUp == RISING ) // in either one of the 2 buttons is released, we release the signal.
    {
        servo[index].commitSettings() ;
    }

// ************ TOGGLE BUTTON, SHORT = TOGGLE STATE, LONG = TOGGLE RELAY ****************
    uint8 time = switches[TOGGLE].pressTime( 2000, 0 ) ;
    if( time == SHORT )
    { 
        waiting4address = 0 ;
        blinkLed(3) ;
        servo[index].manualRelease() ;
        servo[index].setState( !servo[index].getState() ) ;
    }
    else if( time == LONG )
    {
        waiting4address = 0 ;
        servo[index].toggleRelay() ;
        blinkLed( 5 ) ; 
    }

// ************ SELECT, SHORT = SELECT MOTOR, LONG = GET NEW ADDRESS (OR ABORT)
    time = switches[SELECT].pressTime( 2000, 0 ) ;

    if( time == SHORT)
    {
        waiting4address = 0 ;
        blinkLed( 4 ) ;
        if( ++ index == nServos ) index = 0 ;
        servo[index].setState( !servo[index].getState() ) ;
    }

    else if( time == LONG )
    {
        waiting4address ^= 1 ;
        blinkLed(6) ;
    }
}

#elif defined SWITCH_LAYOUT_5BTN
void process5ButtonLayout()
{
    uint8 stateDown = switches[DOWN].state ;
    uint8 stateUp   = switches[ UP  ].state ;

    // ************ FINE TUNING SERVO POSITION ****************
    REPEAT_MS( 200 )
    {
        if( switches[DOWN].state == LOW && switches[UP].state == LOW )
        {
            // eventueel override-mode, indien gewenst
        }
        else if( switches[DOWN].state == LOW )
        {
            waiting4address = 0 ;
            servo[index].decrement() ;
            servo[index].manualRelease() ;
            blinkLed(3) ;
        }
        else if( switches[UP].state == LOW )
        {
            waiting4address = 0 ;
            servo[index].increment() ;
            servo[index].manualRelease() ;
            blinkLed(2) ;
        }
    }
    END_REPEAT

    if( stateDown == RISING || stateUp == RISING )
    {
        servo[index].commitSettings() ;
    }

    // ************ LEFT BUTTON = previous servo + toggle ************
    if( switches[LEFT].pressTime( 0, 0 ) == SHORT )
    {
        if( --index == 255 ) index = nServos - 1 ;
        servo[index].setState( !servo[index].getState() ) ;
        blinkLed(1) ;
    }

    // ************ RIGHT BUTTON = next servo + toggle ************
    if( switches[RIGHT].pressTime( 0, 0 ) == SHORT )
    {
        if( ++index == nServos ) index = 0 ;
        servo[index].setState( !servo[index].getState() ) ;
        blinkLed(4) ;
    }

    // ************ SELECT: SHORT = toggle, LONG = getAddress mode ****
    uint8 time = switches[SEL].pressTime( 1500, 0 ) ;

    if( time == SHORT )
    {
        waiting4address = 0 ;
        servo[index].setState( !servo[index].getState() ) ;
        blinkLed(5) ;
    }
    else if( time == LONG )
    {
        waiting4address = 1 ;
        blinkLed(6) ;
    }
}
#endif

void processSwitches()
{
#if defined SWITCH_LAYOUT_5BTN
    process5ButtonLayout();

#elif defined SWITCH_LAYOUT_4BTN
    process4ButtonLayout();
#endif
}


bool deadbeef()
{
    uint32_t deadbeef_;
    EEPROM.get(DEADBEEF_EE_ADDRESS, deadbeef_);

    if (deadbeef_ != 0xDEADBEEF)
    {
        EEPROM.put(DEADBEEF_EE_ADDRESS, 0xDEADBEEF);
        return true;  // eerste keer boot
    }

    return false;     // al eerder geprogrammeerd
}

void setup()
{
    dcc.pin(2, 0);
    dcc.init(MAN_ID_DIY, 11, FLAGS_OUTPUT_ADDRESS_MODE | FLAGS_DCC_ACCESSORY_DECODER, 0);


#if defined SWITCH_LAYOUT_4BTN
    switches[SELECT].setPin(switchPins[0]);
    switches[TOGGLE].setPin(switchPins[1]);
    switches[    UP].setPin(switchPins[2]);
    switches[  DOWN].setPin(switchPins[3]);

#elif defined SWITCH_LAYOUT_5BTN
    switches[   SEL].setPin(switchPins[0]);
    switches[TOGGLE].setPin(switchPins[1]);
    switches[    UP].setPin(switchPins[2]);
    switches[  DOWN].setPin(switchPins[3]);
    switches[  LEFT].setPin(switchPins[4]);
#endif

    if( deadbeef() )
    {
        EEPROM.put(EE_SETTINGS, defaultSettings );
    }

    EEPROM.get(EE_SETTINGS, settings);

    for (int i = 0; i < nServos; i++)
    {
        servo[i].useEEPROM();
        servo[i].begin(
            servoPins[i],
            defaultMin,
            defaultMax,
            defaultSpeed,
            relayPins[i]
        );
    }
    blinkLed(5);
}

void loop()
{
    statusLed() ;
 
    for (int i = 0; i < nServos; i++)
    {
        servo[i].sweep() ;
    }

    REPEAT_MS( 50 )
    {
        for (int i = 0; i < nSwitches; i++)
        {
            switches[i].debounce() ;
        }
    }
    END_REPEAT

    processSwitches() ;

    dcc.process() ;

    dccConfiguration() ;
}



// Following code adds DCC methods to configure the device.
// the servo sweep library has been slightly altered to work with this method
// servo positions can now be taught in using loco address 9999.
/*  
    You first need to control a servo using it's DCC address
    than you need to enable F0
    than F1, F2 and F3 must be OFF and the speed must be set to 0
    than the servo follows the throttle. 
    With F1 you can store the CURVED position
    With F2 you can store the STRAIGH position
    With F3 you can toggle the relay if it moves in the wrong direction.
    If both F1 and F2 are set you can leave the mode by turning F0 OFF again.
*/

void notifyDccAccTurnoutOutput( uint16 address, uint8 direction, uint8 output )
{
    if( settings.rcn213 == 0 )
    {
        address = constrain( address - 4, 1, 2048 ) ;
    }

    if( waiting4address )
    {   waiting4address = 0 ;

        settings.myAddress = address ;
        EEPROM.put( EE_SETTINGS, settings ) ;
        blinkLed(6);
        return ;
    }

    if( settings.locoFunctions != LOCO_FUNCTIONS_OFF ) return ; // loco functions are used, disable conventional addresses
    //if(  state                 != idle )               return ; // config menu is run, dont set output from OSSD, need fixing
    if( output                 ==    0 )               return ; // ancient DCC flag is a dont care
    if( address <= myAddress || address > myAddress + nServos ) { dccIndex = 0xFF ; return ; }

    index = address - myAddress ;
    dccIndex = index ;

    if( direction >= 1 ) direction = 1 ;

    servo[index].setState( direction ) ;
    blinkLed(3);
}

// CONFIG MODE BY DCC THROTTLE
void notifyDccSpeed( uint16 Addr, DCC_ADDR_TYPE AddrType, uint8 Speed, DCC_DIRECTION Dir, DCC_SPEED_STEPS SpeedSteps)
{
    if( Addr != 9999 ) return ;
    // is speed is negative (dir = 1?, whatever ) speed of 1-128 must be MAP'ed to 20-90 degrees
    // is speed is  postive (dir = 0?, whatever ) speed of 1-128 must be MAP'ed to 90-160 degrees
    servoSpeed = Speed ;
    if( Dir == 1 ) servoSetpoint = map( Speed, 1, 128, 90,  20 ) ;
    else           servoSetpoint = map( Speed, 1, 128, 90, 160 ) ;
}

void notifyDccFunc( uint16 Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8 FuncState)
{
    if( Addr == 9999 ) 
    {
        if( FuncGrp == FN_0_4 )
        {
            F0.update( FuncState & FN_BIT_00 ) ;
            F1.update( FuncState & FN_BIT_01 ) ;
            F2.update( FuncState & FN_BIT_02 ) ;
            F3.update( FuncState & FN_BIT_03 ) ;
            F4.update( FuncState & FN_BIT_04 ) ;
        }
        else if( FuncGrp == FN_5_8 )
        {
            F5.update( FuncState & FN_BIT_05 ) ;
            F6.update( FuncState & FN_BIT_06 ) ;
            F7.update( FuncState & FN_BIT_07 ) ;
            F8.update( FuncState & FN_BIT_08 ) ;
        }
        else return ;
    }

    // Locomotive function support
    if( settings.locoFunctions == LOCO_FUNCTIONS_OFF ) return ; // loco functions are NOT used
    if( Addr                   != settings.myAddress ) return ; // not my address

    static uint16_t oldState = 0 ;
    static uint16_t newState = 0 ;

    uint8_t  bitOffset       = 0 ;
    uint16_t dccBaseAddress  = settings.myAddress ;
    uint8_t  neededAddresses = 1 ;                      // used for the Lok maus support. As we have 4 dcc addresses per loco address, we need more loco addresses per decoder 

    if( settings.locoFunctions == FANTASTIC_FOUR )      // lok maus support, F1-F4 of sequential loco addresses are used to control a decoder 
    {
        if( FuncGrp != FN_0_4 ) return ;

        neededAddresses = (nServos - 1) / 4 + 1 ; 
        if (Addr < dccBaseAddress || Addr >= (dccBaseAddress + neededAddresses)) return ;

        bitOffset = (Addr - dccBaseAddress) * 4 ;
    }

    if( settings.locoFunctions == EIGHT_BALL ) // 1 loco address 16 functions max (if single outputs are used.
    {
        if (Addr != dccBaseAddress) return ;

        if(      FuncGrp ==   FN_0_4 ) { bitOffset =  0 ; }
        else if( FuncGrp ==   FN_5_8 ) { bitOffset =  4 ; }
        else if( FuncGrp ==  FN_9_12 ) { bitOffset =  8 ; }
        else if( FuncGrp == FN_13_20 ) { bitOffset = 12 ; }
        else return ;
    }

    newState &= ~(0x0F << bitOffset) ;              // first clear the old bits
    newState |= (FuncState & 0x0F) << bitOffset ;   // than reload the newly received bits

    uint16_t changedBits    = oldState ^ newState;  // if atleast 1 bit has changed, 
    oldState                = newState ;
    if( changedBits == 0 )  return ;

    for( int i = 0 ; i < nServos ; i++ )            // loop through all changedBits to set the coil.
    {
        uint16_t andMask = 1 << i ;
        if(( changedBits & andMask ) == 0 ) continue ;

        uint8_t  state = (newState >> i) & 0x1 ;
        servo[i].setState( state ) ;
        blinkLed(3);
    }
}

void dccConfiguration()
{
    if( dccIndex == 0xFF )
    {
        state = idle ;
        return ;
    }

    switch( state )
    {
    case idle:
        if( F0.Q    // if F0 turned ON, speed = 0 and all other functions are OFF
        &&  !F1.EN 
        &&  !F2.EN 
        &&  !F3.EN 
        &&  !F4.EN 
        &&  !F5.EN 
        &&  !F6.EN 
        &&  !F7.EN 
        &&  !F8.EN 
        && servoSetpoint == 90 ) state = configureServo ;
        break ;


        
    case configureServo:
        // NOTE BRAIN FART, IF F8 is active, you may use the active speed to keep toggling the motor while the throttle can be used for speed.

        if( !F8.EN )
        {
            REPEAT_MS( 50 ) // update servo position
            {
                servo[dccIndex].manualOverride( servoSetpoint ) ;
            }
            END_REPEAT
        } else
        {
            REPEAT_MS( 3000 )
            {
                servo[dccIndex].setSpeed( servoSpeed ) ;
                servo[dccIndex].setState( !servo[dccIndex].getState() ) ;
            }
            END_REPEAT
        }

        if( F1.Q ) { servo[dccIndex].setMin( servoSetpoint ) ;   blinkLed( 2 ) ; }
        if( F2.Q ) { servo[dccIndex].setMax( servoSetpoint ) ;   blinkLed( 3 ) ; }
        if( F3.Q ) { servo[dccIndex].toggleRelay() ;             blinkLed( 4 ) ; }

        if( F4.EN ) { settings.locoFunctions = LOCO_FUNCTIONS_OFF ;  }
        if( F5.EN ) { settings.locoFunctions = FANTASTIC_FOUR ;  }
        if( F6.EN ) { settings.locoFunctions = EIGHT_BALL ;  }
        settings.rcn213 = F7.EN ;

        if( !F0.EN )
        {
            servo[dccIndex].manualRelease() ;
            servo[dccIndex].commitSettings() ;
            EEPROM.put( EE_SETTINGS, settings ) ;

            state = idle ;
        }
        
        break ;
    }
}