#include <Arduino.h>

const uint8_t servoPins[]  = {  0,  1,  5,  6,  9, 10 } ;
const uint8_t relayPins[]  = {  3,  4,  7,  8, A0, A1 } ;
const uint8_t switchPins[] = { A2, A3, A4, A5, 255 } ; // 255 means NOT in ushe 

const uint8_t nServos   = sizeof(  servoPins ) / sizeof(  servoPins[0] ) ;
const uint8_t nSwitches = sizeof( switchPins ) / sizeof( switchPins[0] ) ;

//#define SWITCH_LAYOUT_5BTN
#define SWITCH_LAYOUT_4BTN


#if defined SWITCH_LAYOUT_5BTN
  enum { LEFT, RIGHT, UP, DOWN, SEL } ;

#elif defined  SWITCH_LAYOUT_4BTN
  enum { TOGGLE, SELECT, UP, DOWN  } ;
#else
  #error "Unknown SWITCH_LAYOUT"
#endif