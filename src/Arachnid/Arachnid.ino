/* Arachnid -- Main Driver
 * Larson Rivera (a.k.a bredbord)
 * Last Modified: 11/14/2020
 * Version 3.0
*/

// Libraries
#include <TeensyDMX.h>
#include <OctoWS2811.h>
#include <SafetyStepperArray.h>

#include "config.h"

// Timers---------
elapsedMillis lastFrameTimer;  // when last DMX frame was received -- for float Lock
elapsedMillis floatLockTimer;  // timer for the float lock
elapsedMillis lightUpdate;

bool floatLock = false;

// HARDWARE SETUP==============================================================
SafetyStepperArray cardinal = SafetyStepperArray(ENABLE_PIN, SLEEP_PIN, 1000 * MICROSTEPS, 1000* MICROSTEPS);

// Onboard pins--------------------------------------
constexpr uint8_t statusLEDPin = STATUS_LED_PIN;
uint8_t statusLEDBrightness = 0;


// LEDS-----------------------------------------------

// These buffers need to be large enough for all the pixels.
// The total number of pixels is "ledsPerStrip * numPins".
// Each pixel needs 3 bytes, so multiply by 3.  An "int" is
// 4 bytes, so divide by 4.  The array is created using "int"
// so the compiler will align it to 32 bit memory.
DMAMEM int displayMemory[NUM_LEDS * 3 / 4];
int drawingMemory[NUM_LEDS * 3 / 4];

const int config = WS2811_GBR | WS2811_800kHz;
OctoWS2811 leds(LEDS_PER_FIXTURE, displayMemory, drawingMemory, config, NUM_LED_FIXTURES, LED_PIN_LIST);

byte ledColors[NUM_LEDS][3]{0};  // data for holding each pixel's data


// COLOR ENCODING AND CORRECTION----------------------

//encoding
struct RGB {
  byte red;
  byte green;
  byte blue;
};

// DMX -----------------------------------------------
namespace teensydmx = ::qindesign::teensydmx;
teensydmx::Receiver dmxRx{Serial1};
uint8_t DMXData[DMX_LENGTH]{0};
uint8_t DMXInput[DMX_LENGTH]{0};  // future buffering

constexpr unsigned long kDMXTimeout = 100;  // Millis for considered DMX timeout
constexpr unsigned long kFloatLockTimeout = 2000;  // Millis for Floating Lock timeout

//END HARDWARE SETUP =================================================================



// PROTOTYPES-----------------------------------------

//LEDS
void setBarColor(int, byte, byte, byte, byte);
void setBarColor(int, byte, int);

void setFixtureColor(byte, byte, byte, byte);
void setFixtureColor(byte, int);

void setAllColor(byte, byte, byte);
void setAllColor(int);

void setBarTemperature(int, byte, int);
void setFixturetemperature(byte, int);
void setAllTemperature(int);

RGB hexToRGB(int);


//DMX
bool updateDMX();
void updateLEDDMX();
void updateStepperDMX();

//SYSTEM
void stopWithError();


// ================================================================================
// ==================================BEGIN MAIN====================================
// ================================================================================
void setup() {
  
  // HARD CALL PIN SETUP-------------------------------
  pinMode(statusLEDPin, OUTPUT);

  // Cardinal Setup------------------------------------
  cardinal.addStepper(10, 9, 11);
  cardinal.addStepper(24, 12, 25);
  cardinal.addStepper(27, 26, 28);
  cardinal.addStepper(30, 29, 31);
  cardinal.addStepper(23, 22, 19);
  cardinal.addStepper(18, 17, 40);
  cardinal.addStepper(39, 38, 37);
  cardinal.addStepper(36, 25, 34);

  cardinal.begin();

  
  // LED SETUP-----------------------------------------
  leds.begin();
  setAllColor(255, 0, 127);
  leds.show();

  // DMX SETUP-----------------------------------------
  dmxRx.begin();
  lastFrameTimer = kDMXTimeout;


  // Hardware Calibration------------------------------
  if (!cardinal.homeSteppers(1,1,10000)) stopWithError();
}

// MAIN++++++++++++++++++++++++++++++++++++++++++++++++
void loop() {
  int read = dmxRx.readPacket(DMXData, DMX_START, DMX_START + DMX_LENGTH);
  if (read > 0) lastFrameTimer = 0;  // some DMX data received

  // DMX------------------------------------
  if (!floatLock && lastFrameTimer <= kDMXTimeout) { // if we are not in floatLock, and the DMX connection is valid
    floatLockTimer = 0;  // reset float locking timer
    
    // Do when receiving DMX=======================
    
    cardinal.setTimeoutMillis(10000);
    updateStepperDMX();
    updateLEDDMX();
  } 

  // NO DMX---------------------------------
  else {
    // DMX Float Locking
    if (!floatLock && floatLockTimer < kDMXTimeout * 2) floatLock = true; // if the floatLock is not enabled, and the disconection just started, enable the lock
    if (floatLock && floatLockTimer > kFloatLockTimeout) floatLock = false;  // if we are in floatLock, and the locking timer has expired, release the lock

    // Do when not receiving DMX=====================
    
    cardinal.setTimeoutMillis(3000);
    for (short s = 1; s <= NUM_STEPPERS; s++) { cardinal.setStepperSpeed(s, 1000 * MICROSTEPS); cardinal.setStepperPosition(s, 0); }
    
  }
  

  // STEPPER UPDATING----------------------
  cardinal.run();
  
}

// ================================================================================
// ==================================END OF MAIN===================================
// ================================================================================



// ================================================================================
// =============================ADDITIONAL FUNCTIONS===============================
// ================================================================================


// LEDS===============================================
void setBarColor(int bar, byte fixture, byte r, byte g, byte b) {
  
  int ledOffset; bar--; fixture--;
  for (int pixel = 0; pixel < LEDS_PER_BAR; pixel++) { // for each pixel in the bar
    ledOffset = (fixture * LEDS_PER_FIXTURE) + (bar * LEDS_PER_BAR) + pixel;   //calculate led position
    leds.setPixel(ledOffset, r, g, b);                                       // update the pixel
  }
}
void setBarColor(int bar, byte fixture, int c) {
  RGB tripple = hexToRGB(c);
  setBarColor(bar, fixture, tripple.red, tripple.green, tripple.blue);
}

void setFixtureColor(byte fixture, byte r, byte g, byte b) {
  for (int ba = 1, pix =0; ba <= BARS_PER_FIXTURE && pix < LEDS_PER_FIXTURE; ba++, pix+= LEDS_PER_BAR) { // for each bar in the fixture while we don't exceed the bars
    setBarColor(ba, fixture, r, g, b);  // set each bar color
  }
}
void setFixtureColor(byte fixture, int c) {
  RGB tripple = hexToRGB(c);
  setFixtureColor(fixture, tripple.red, tripple.green, tripple.blue);
}

void setAllColor(byte r, byte g, byte b) {
  for (byte f = 1; f <= NUM_LED_FIXTURES; f++) {
    setFixtureColor(f, r, g, b);
  }
}
void setAllColor(int c) {
  RGB tripple = hexToRGB(c);
  setAllColor(tripple.red, tripple.green, tripple.blue);
}

void setBarTemperature(int bar, byte fixture, int temp) {
  setBarColor(bar, fixture, temp + TEMPERATURE_OFFSET);
}
void setFixtureTemperature(byte fixture, int temp) {
  setFixtureColor(fixture, temp + TEMPERATURE_OFFSET);
}
void setAllTemperature(int temp) {
  setAllColor(temp + TEMPERATURE_OFFSET);
}

RGB hexToRGB(int hex) {
  RGB t = {
    (hex >> 16) & 0xFF, // RR byte
    (hex >> 8) & 0xFF, // GG byte
    (hex) & 0xFF // BB byte
  };

  return t;
}




// DMX================================================
void updateLEDDMX() {
  int DMXOffset;  // offset for DMX data
    
  for (int f = 1; f <= NUM_LED_FIXTURES; f++) {                 // for each fixture
    for (int b = 1; b <= BARS_PER_FIXTURE; b++) {                       // for each bar in the fixture
  
        //DMXOffset = (((b-1) * OPERATIONS_PER_BAR) + OPERATIONS_PER_STEPPER) * f;  // and dmx data location
        DMXOffset = (NUM_STEPPERS * OPERATIONS_PER_STEPPER) + ((b-1) * OPERATIONS_PER_BAR) * f;
        setBarColor(b, f, DMXData[DMXOffset], DMXData[DMXOffset+1], DMXData[DMXOffset+2]);  // set bar b at fixture f to the DMX values
    }
  }
}

void updateStepperDMX() {
  int speedTarget, motionTarget;
  byte DMXOffset;
  
  for (short s = 0; s < NUM_STEPPERS; s++) {
    //DMXOffset = (s * OPERATIONS_PER_FIXTURE);  // calculate the DMX data location
    DMXOffset = (OPERATIONS_PER_STEPPER * s);
    // Speed Section-------------------------------------------------------
    if(DMXData[DMXOffset + 1] == 0) speedTarget = DEFUALT_MAX_SPEED;
    else speedTarget = map(DMXData[DMXOffset+1], 1, 255, ABS_STEPPER_MIN_SPEED, ABS_STEPPER_MAX_SPEED);
    cardinal.setStepperSpeed(s+1, speedTarget);

    // Motion Section------------------------------------------------------
    motionTarget = DMXData[DMXOffset] * MICROSTEPS*4;  // get the new stepper position
    cardinal.setStepperPosition(s+1, motionTarget);
  }
}



// SYSTEM============================================
void stopWithError() {
  setAllColor(255, 0, 0);
  leds.show();

  cardinal.emergencyStop();
  
}
