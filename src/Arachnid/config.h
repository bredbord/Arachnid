/* Arachnid -- Configuration Header
 * Larson Rivera (a.k.a bredbord)
 * Last Modified: 11/14/2020
 * Version 3.0
*/

// BASIC SETUP============================================================

// DMX--------------------
  #define DMX_START 1

// Fixtures
  #define NUM_FIXTURES 8
  
// STEPPERS---------------
  #define NUM_STEPPERS (NUM_FIXTURES)
  #define NUM_SWITCHES (NUM_STEPPERS)
  #define MICROSTEPS 16
  #define DEFUALT_SPEED 3000
  #define DEFAULT_ACCELERATION 3000
  #define STEPPER_MAX_SPEED 3000
  #define STEPPER_MIN_SPEED 100

// LEDs--------------------
  #define LEDS_PER_FIXTURE 12
  #define BARS_PER_FIXTURE 4
  #define REFRESH_RATE_HZ 60
  // Pin layouts on the teensy 3:
  /*
   Required Connections
  --------------------
    pin 2:  LED Strip #1    OctoWS2811 drives 8 LED Strips.
    pin 14: LED strip #2    All 8 are the same length.
    pin 7:  LED strip #3
    pin 8:  LED strip #4    A 100 ohm resistor should used
    pin 6:  LED strip #5    between each Teensy pin and the
    pin 20: LED strip #6    wire to the LED strip, to minimize
    pin 21: LED strip #7    high frequency ringining & noise.
    pin 5:  LED strip #8
    pin 15 & 16 - Connect together, but do not use
    pin 4 - Do not use
    pin 3 - Do not use as PWM.  Normal use is ok.
    */

//ADVANCED================================================================

// DMX-------------------------
#define DMX_LENGTH ((BARS_PER_FIXTURE * 3) * NUM_FIXTURES) + NUM_STEPPERS
#define OPERATIONS_PER_BAR 3
#define OPERATIONS_PER_FIXTURE (OPERATIONS_PER_STEPPER + (OPERATIONS_PER_BAR * BARS_PER_FIXTURE))  // assumes stepper-> light priority


// STEPPERS--------------------
#define OPERATIONS_PER_STEPPER 2

#define STEPPER_DMX_TIMEOUT 30000
#define STEPPER_STANDALONE_TIMEOUT 10000

#define DEFUALT_MAX_SPEED (DEFUALT_SPEED * MICROSTEPS)
#define DEFUALT_MAX_ACCELERATION (DEFAULT_ACCELERATION * MICROSTEPS)
#define ABS_STEPPER_MAX_SPEED (STEPPER_MAX_SPEED * MICROSTEPS)
#define ABS_STEPPER_MIN_SPEED (STEPPER_MIN_SPEED * MICROSTEPS)

#define STEPPER_SLEEP_PIN 33
#define STEPPER_ENABLE_PIN 32

#define MOTION_TIMEOUT_DURATION 3000

// LED-------------------------

#define LED_REFRESH (1000000 / REFRESH_RATE_HZ)
#define LEDS_PER_BAR (LEDS_PER_FIXTURE / BARS_PER_FIXTURE)

// SYSTEM----------------------
#define STATUS_LED_PIN LED_BUILTIN

#define LED_DMX_RX_ACTIVE 127
#define LED_DMX_RX_SLEEP 10
#define LED_STDBY 0
