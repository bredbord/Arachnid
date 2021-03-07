/* Arachnid -- Main Driver
 * Larson Rivera (a.k.a bredbord)
 * Last Modified: 11/14/2020
 * Version 3.0
*/

// Libraries
#include <TeensyDMX.h>
#include <AccelStepper.h>
#include <OctoWS2811.h>

#include "config.h"

// Timers---------
elapsedMillis lastFrameTimer;  // when last DMX frame was received -- for float Lock
elapsedMillis floatLockTimer;  // timer for the float lock
elapsedMillis lastStepperChange;    // when last stepper change
elapsedMillis lastDMXChange;  // when the DMX last changed
elapsedMillis lightUpdate; 

bool floatLock = false;

// HARDWARE SETUP==============================================================

// Onboard pins--------------------------------------
constexpr uint8_t statusLEDPin = STATUS_LED_PIN;
uint8_t statusLEDBrightness = 0;

// Steppers-------------------------------------------
AccelStepper M1(1,10,9), M2(1,24,12), M3(1,27,26), M4(1,30,29), M5(1,23,22), M6(1,18,17), M7(1,39,38), M8(1,36,35);
AccelStepper *stepper[NUM_STEPPERS] = {&M1, &M2, &M3, &M4, &M5, &M6, &M7, &M8};
int stepperPositions[NUM_STEPPERS]{0};
int newStepperPositions[NUM_STEPPERS]{0};
bool steppersEnabled;

// Buttons--------------------------------------------
int limitPins[NUM_SWITCHES] = {11, 25, 28, 31, 19, 40, 37, 34};

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

// LED COLOR TEMP CORRECTIONS from FastLED

enum LEDColorCorrection {
  TypicalSMD5050 =0xFFB0F0, TypicalLEDStrip =0xFFB0F0, Typical8mmPixel =0xFFE08C, TypicalPixelString =0xFFE08C,
  UncorrectedColor =0xFFFFFF
};

enum ColorTemperature {
  Candle =0xFF9329, Tungsten40W =0xFFC58F, Tungsten100W =0xFFD6AA, Halogen =0xFFF1E0,
  CarbonArc =0xFFFAF4, HighNoonSun =0xFFFFFB, DirectSunlight =0xFFFFFF, OvercastSky =0xC9E2FF,
  ClearBlueSky =0x409CFF, WarmFluorescent =0xFFF4E5, StandardFluorescent =0xF4FFFA, CoolWhiteFluorescent =0xD4EBFF,
  FullSpectrumFluorescent =0xFFF4F2, GrowLightFluorescent =0xFFEFF7, BlackLightFluorescent =0xA700FF, MercuryVapor =0xD8F7FF,
  SodiumVapor =0xFFD1B2, MetalHalide =0xF2FCFF, HighPressureSodium =0xFFB74C, UncorrectedTemperature =0xFFFFFF
};

// DMX -----------------------------------------------
namespace teensydmx = ::qindesign::teensydmx;
teensydmx::Receiver dmxRx{Serial1};
uint8_t DMXData[DMX_LENGTH]{0};
uint8_t DMXInput[DMX_LENGTH]{0};  // future buffering

constexpr unsigned long kDMXTimeout = 100;  // Millis for considered DMX timeout
constexpr unsigned long kFloatLockTimeout = 2000;  // Millis for Floating Lock timeout
unsigned long kStepperTimeout;  // Millis for stepper timeout

//END HARDWARE SETUP =================================================================



// PROTOTYPES-----------------------------------------

//Steppers
bool homeFixtures(byte, byte, int);   // attempts to home all fixtures
void enableSteppers(bool);
void runSteppers();                   // updates stepper positions
bool allHome();                       // reports if all fixtures are home

//LEDS
void setBarColor(int, byte, byte, byte, byte);
void setFixtureColor(byte, byte, byte, byte);
void setAllColor(byte, byte, byte);

void setBarColor(int, byte, int);
void setFixtureColor(byte, int);
void setAllColor(int);

RGB getRGBColor(int);

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
  pinMode(STEPPER_SLEEP_PIN, OUTPUT);
  pinMode(STEPPER_ENABLE_PIN, OUTPUT);

  // STEPPER SETUP-------------------------------------
  for (short i = 0; i < NUM_STEPPERS; i++) { stepper[i]->setMaxSpeed(DEFUALT_MAX_SPEED); stepper[i]->setAcceleration(DEFUALT_MAX_ACCELERATION); }  // Set default speed and accel
  enableSteppers(true);
  
  // LIMIT SWITCH SETUP--------------------------------
  for (byte i = 0; i < NUM_SWITCHES; i++) { pinMode(limitPins[i], INPUT_PULLUP); }

  // LED SETUP-----------------------------------------
  leds.begin();
  setAllColor(255, 0, 127);
  leds.show();

  // DMX SETUP-----------------------------------------
  dmxRx.begin();
  lastFrameTimer = kDMXTimeout;


  // Hardware Calibration------------------------------
  if (!homeFixtures(1,1, 10000)) stopWithError();
}

// MAIN++++++++++++++++++++++++++++++++++++++++++++++++
void loop() {
  int read = dmxRx.readPacket(DMXInput, DMX_START, DMX_START + DMX_LENGTH);
  if (read > 0) lastFrameTimer = 0;  // some DMX data received

  // DMX------------------------------------
  if (!floatLock && lastFrameTimer <= kDMXTimeout) { // if we are not in floatLock, and the DMX connection is valid
    floatLockTimer = 0;  // reset float locking timer
    kStepperTimeout = STEPPER_DMX_TIMEOUT; //set stepper timeout to 30 seconds

    // STEPPERS-------------------------------------------------
    if (updateDMX()) { lastDMXChange = 0; statusLEDBrightness = 127; }  // if we see a DMX change, reset the counter
    statusLEDBrightness = LED_DMX_RX_ACTIVE;
    
    if (lastDMXChange < kStepperTimeout - MOTION_TIMEOUT_DURATION) updateStepperDMX();  // if there is new DMX within the pre-time-out, update based on DMX
    else { statusLEDBrightness = LED_DMX_RX_SLEEP; }
    // otherwise, we are't seeing any new data from lighting software, so ignore DMX, move to nearest holding lock, disable to save power, and hold there until new DMX

    // LEDS-----------------------------------------------------
    updateLEDDMX();
  } 

  // NO DMX---------------------------------
  else {
    // DMX Float Locking
    if (!floatLock && floatLockTimer < kDMXTimeout * 2) floatLock = true; // if the floatLock is not enabled, and the disconection just started, enable the lock
    if (floatLock && floatLockTimer > kFloatLockTimeout) floatLock = false;  // if we are in floatLock, and the locking timer has expired, release the lock

    statusLEDBrightness = LED_STDBY;
    analogWrite(statusLEDPin, statusLEDBrightness);

    // STEPPERS-------------------------------------------------
    if (allHome()) kStepperTimeout = 500; //if everything is at its endstops, timeout doesn't need to take forever.
    else kStepperTimeout = STEPPER_STANDALONE_TIMEOUT; //set stepper timeout to 10 seconds

    // LEDS-----------------------------------------------------
    setAllColor(Candle + TypicalLEDStrip);
  }

  analogWrite(statusLEDPin, statusLEDBrightness);
  

  // STEPPER UPDATING----------------------
  runSteppers();

  // LED UPDATING--------------------------
  if (lightUpdate > LED_REFRESH_MILLIS) {
    leds.show(); lightUpdate = 0;
  }
}

// ================================================================================
// ==================================END OF MAIN===================================
// ================================================================================



// ================================================================================
// =============================ADDITIONAL FUNCTIONS===============================
// ================================================================================

// STEPPERS===========================================
bool homeFixtures(byte startStep, byte stopStep, int homeTimer) {
  
  startStep--; stopStep--;
  unsigned long currentTime = millis();  // time of homing start
  bool allHome;  // all home flag

  for (byte s = startStep; s <= stopStep; s++) stepper[s]->setSpeed(-300*MICROSTEPS);  // set all steppers to run backward

  do {
    allHome = true;  // assume home unless proven otherwise
    
    for (byte s = startStep; s <= stopStep; s++) {  // check each stepper
      if (digitalRead(limitPins[s]) == LOW) stepper[s]->setCurrentPosition(0);  // for triggered limit switch
      else { stepper[s]->runSpeed(); allHome = false; }
    }
    
  } while (!allHome && millis() - currentTime < homeTimer);  // while all fixtures are not homed, and we are within the alotted time

  for (byte s = startStep; s <= stopStep; s++) stepper[s]->setSpeed(0);  // Zero stepper speed. 

  if (allHome) {
    for (byte s = startStep; s <= stopStep; s++) stepperPositions[s] = 0;  // Zero out stepper positions.
    return true;  // return success
  }

  else return false;  // otherwise, we timed out.
}

void enableSteppers(bool state) {
  if (state) {
    digitalWrite(STEPPER_SLEEP_PIN, HIGH);
    digitalWrite(STEPPER_ENABLE_PIN, LOW);
    steppersEnabled = true;
  } else {
    digitalWrite(STEPPER_SLEEP_PIN, LOW);
    digitalWrite(STEPPER_ENABLE_PIN, HIGH);
    steppersEnabled = false;
  }
}

void runSteppers() {
  int currentPos, newPos;

  // check for any updates to position, update positions, and reset timeout if any occur
  for (int s = 0; s < NUM_STEPPERS; s++) {
    currentPos = stepperPositions[s];
    newPos = newStepperPositions[s];  // updates continously* based on DMX
    
    if (currentPos != newPos) {  // if we have a new position request
      stepperPositions[s] = newPos; // update position index
      lastStepperChange = 0; // reset the timeout
    }
  }

  // If we have not timed out-------------------------------
  if (lastStepperChange < kStepperTimeout) {
    if (!steppersEnabled) { enableSteppers(true); delay(10); }  // if the steppers are not enabled, enable and allow hardware catch up

    if (lastStepperChange > kStepperTimeout - MOTION_TIMEOUT_DURATION) { 
      int lockVal;
      for (int h = 0; h < NUM_STEPPERS; h++) {
        lockVal = 0;
        stepperPositions[h] = lockVal;
        newStepperPositions[h] = lockVal;
      }
    }
    
    for (byte m = 0; m < NUM_STEPPERS; m++) { stepper[m]->moveTo(stepperPositions[m]); }
    for (byte m = 0; m < NUM_STEPPERS; m++) { stepper[m]->run(); }
  } 

  // Otherwise--------------------------------
   else {
    enableSteppers(false);
  }
}

bool allHome() {
  for (byte s = 0; s <= NUM_STEPPERS; s++) {  // check each stepper
    if (!digitalRead(limitPins[s]) == LOW) return false;  // for triggered limit switch
  }
  return true;
}



// LEDS===============================================
void setBarColor(int bar, byte fixture, byte r, byte g, byte b) {
  
  int ledOffset; bar--; fixture--;
  for (int pixel = 0; pixel < LEDS_PER_BAR; pixel++) { // for each pixel in the bar
    ledOffset = (fixture * LEDS_PER_FIXTURE) + (bar * LEDS_PER_BAR) + pixel;   //calculate led position
    leds.setPixel(ledOffset, r, g, b);                                       // update the pixel
  }
}
void setBarColor(int bar, byte fixture, int hex) {
  RGB c =  getRGBColor(hex);
  setBarColor(bar, fixture, c.red, c.green, c.blue);
}

void setFixtureColor(byte fixture, byte r, byte g, byte b) {
  for (int ba = 1; ba <= BARS_PER_FIXTURE; ba++) { // for each bar in the fixture
    setBarColor(ba, fixture, r, g, b);  // set each bar color
  }
}
void setFixtureColor(byte fixture, int hex) {
  RGB c =  getRGBColor(hex);
  setFixtureColor(fixture, c.red, c.green, c.blue);
}

void setAllColor(byte r, byte g, byte b) {
  for (byte f = 1; f <= NUM_LED_FIXTURES; f++) {
    setFixtureColor(f, r, g, b);
  }
}
void setAllColor(int hex){
  RGB c =  getRGBColor(hex);
  setAllColor(c.red, c.green, c.blue);
}

RGB getRGBColor(int h) {
  RGB rgb =  {
    ((h >> 16) & 0xFF) / 255.0,
    ((h >> 8) & 0xFF) / 255.0,
    ((h) & 0xFF) / 255.0
  };

  return rgb;
}


// DMX================================================
bool updateDMX() {
  bool updateFlag = false;
  uint8_t current, compare;
  
  // Compare the array
  for (int c = 0; c < DMX_LENGTH; c++) {
    current = DMXData[c]; compare = DMXInput[c];
    if (current != compare) { DMXData[c] = compare; updateFlag = true; }  // if there was a change, note it and trigger update flag
  }

  return updateFlag;
}

void updateLEDDMX() {
  int DMXOffset;  // offset for DMX data
    
  for (int f = 1; f <= NUM_LED_FIXTURES; f++) {                 // for each fixture
    for (int b = 1; b <= BARS_PER_FIXTURE; b++) {                       // for each bar in the fixture
  
        DMXOffset = (((b-1) * OPERATIONS_PER_BAR) + OPERATIONS_PER_STEPPER) * f;  // and dmx data location
        setBarColor(b, f, DMXData[DMXOffset], DMXData[DMXOffset+1], DMXData[DMXOffset+2]);  // set bar b at fixture f to the DMX values
    }
  }
}

void updateStepperDMX() {
  int speedTarget, motionTarget;
  byte DMXOffset;
  
  for (short s = 0; s < NUM_STEPPERS; s++) {
    DMXOffset = (s * OPERATIONS_PER_FIXTURE);  // calculate the DMX data location

    // Speed Section-------------------------------------------------------
    if(DMXData[DMXOffset + 1] == 0) speedTarget = DEFUALT_MAX_SPEED;
    else speedTarget = map(DMXData[DMXOffset+1], 1, 255, ABS_STEPPER_MIN_SPEED, ABS_STEPPER_MAX_SPEED);
    stepper[s]->setMaxSpeed(speedTarget);  // map the acceleration from the DMX input

    // Motion Section------------------------------------------------------
    motionTarget = DMXData[DMXOffset] * MICROSTEPS*4;  // get the new stepper position
    newStepperPositions[s] = motionTarget;
  }
}



// SYSTEM============================================
void stopWithError() {
  enableSteppers(false);
  
  while (true) {
    analogWrite(statusLEDPin, 1023);
    setAllColor(255, 0, 0);
    leds.show();
    
    delay(1000);
    
    analogWrite(statusLEDPin, 0);
    setAllColor(50, 0, 0);
    leds.show();
    
    delay(1000);
  }
  
}
