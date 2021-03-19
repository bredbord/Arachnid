/* Arachnid -- Configuration Header
 * Larson Rivera (a.k.a bredbord)
 * Last Modified: 11/14/2020
 * Version 3.0
*/

#include "color.h"

// BASIC SETUP============================================================

// DMX--------------------
  #define DMX_START 1

// Fixtures
  #define NUM_MOTION_FIXTURES 8  // number of fixture nodes with motion capability
  #define NUM_LED_FIXTURES 4    // number of fixture nodes with LED capability
  
  
// STEPPERS---------------
  #define NUM_STEPPERS (NUM_MOTION_FIXTURES)
  #define NUM_SWITCHES (NUM_STEPPERS)
  #define MICROSTEPS 16
  #define DEFUALT_SPEED 500
  #define DEFAULT_ACCELERATION 250
  #define STEPPER_MAX_SPEED 500
  #define STEPPER_MIN_SPEED 1

// LEDs--------------------
  #define LEDS_PER_FIXTURE 12
  #define BARS_PER_FIXTURE 6
  #define TEMPERATURE_OFFSET TypicalSMD5050

  const byte LED_PIN_LIST[NUM_LED_FIXTURES] = {2, 14, 7, 8};  // sequencing of Teensy LED Pins

    // Pin layouts on Arachnid Mainboard:
  /*
    DEFUALT CONFIGURATION
    pin 2:  LED Strip #1
    pin 14: LED strip #2   
    pin 7:  LED strip #3
    pin 8:  LED strip #4   
    pin 6:  LED strip #5   
    pin 20: LED strip #6    
    pin 21: LED strip #7    
    pin 5:  LED strip #8
  */

//ADVANCED================================================================

// DMX-------------------------
#define OPERATIONS_PER_BAR 3
#define OPERATIONS_PER_FIXTURE (OPERATIONS_PER_STEPPER + (OPERATIONS_PER_BAR * BARS_PER_FIXTURE))  // assumes stepper-> light priority

// STEPPERS--------------------
#define STEPPERS_PER_FIXTURE 1
#define OPERATIONS_PER_STEPPER 2

#define STEPPER_DMX_TIMEOUT 30000
#define STEPPER_STANDALONE_TIMEOUT 10000

#define DEFUALT_MAX_SPEED (DEFUALT_SPEED * MICROSTEPS)
#define DEFUALT_MAX_ACCELERATION (DEFAULT_ACCELERATION * MICROSTEPS)
#define ABS_STEPPER_MAX_SPEED (STEPPER_MAX_SPEED * MICROSTEPS)
#define ABS_STEPPER_MIN_SPEED (STEPPER_MIN_SPEED * MICROSTEPS)

#define SLEEP_PIN 33
#define ENABLE_PIN 32

#define MOTION_TIMEOUT_DURATION 3000

// LED-------------------------

#define NUM_LEDS (LEDS_PER_FIXTURE * NUM_LED_FIXTURES)
#define LEDS_PER_BAR (LEDS_PER_FIXTURE / BARS_PER_FIXTURE)
#define LED_REFRESH_MILLIS 16

// SYSTEM----------------------
#define STATUS_LED_PIN LED_BUILTIN

#define DMX_LENGTH (((BARS_PER_FIXTURE * OPERATIONS_PER_BAR) * NUM_LED_FIXTURES) + (NUM_STEPPERS * OPERATIONS_PER_STEPPER))

#define LED_DMX_RX_ACTIVE 127
#define LED_DMX_RX_SLEEP 10
#define LED_STDBY 0

#define HOME_COLOR 0X00FF11
#define NON_HOME_COLOR 0x006EFF
