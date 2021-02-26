/* Arachnid -- Main Driver
 * Larson Rivera (a.k.a bredbord)
 * Last Modified: 11/14/2020
 * Version 3.0
*/

// Libraries
#include <TeensyDMX.h>
#include <AccelStepper.h>
#include "config.h"

// HARDWARE SETUP==============================================================

// DMX -----------------------------------------------
namespace teensydmx = ::qindesign::teensydmx;
teensydmx::Receiver dmxRx{Serial1};
uint8_t DMXData[DMX_LENGTH]{0};
uint8_t lastDMX[DMX_LENGTH]{0};

constexpr unsigned long kDMXTimeout = 100;  // Millis for considered DMX timeout
constexpr unsigned long kFloatLockTimeout = 2000;  // Millis for Floating Lock timeout

// Keeps track of when the last frame was received.
elapsedMillis lastFrameTimer;
elapsedMillis floatLockTimer;
bool floatLock = false;

// The LED pin.
constexpr uint8_t kLEDPin = LED_BUILTIN;


// Steppers-------------------------------------------
AccelStepper M1(1,10,9), M2(1,24,12), M3(1,27,26), M4(1,30,29), M5(1,23,22), M6(1,18,17), M7(1,39,38), M8(1,36,35);
AccelStepper *stepper[NUM_STEPPERS] = {&M1, &M2, &M3, &M4, &M5, &M6, &M7, &M8};

// PROTOTYPES-----------------------------------------

bool compareDMXArrays();

void setup() {
  // Set up any pins
  pinMode(kLEDPin, OUTPUT);

  dmxRx.begin();
  lastFrameTimer = kDMXTimeout;
}

void loop() {
  int read = dmxRx.readPacket(DMXInput, DMX_START, DMX_START + DMX_LENGTH);
  if (read > 0) {
    // We've read everything we want to
    lastFrameTimer = 0;
  }

  // DMX------------------------------------
  if (!floatLock && lastFrameTimer <= kDMXTimeout) { // if we are not in floatLock, and the DMX is valid
    floatLockTimer = 0;  // reset float locking timer
    analogWrite(kLEDPin, DMXInput[0]);
  } 

  // NO DMX---------------------------------
  else {
    // DMX Float Locking
    if (!floatLock && floatLockTimer < kDMXTimeout * 2) floatLock = true; // if the floatLock is not enabled, and the disconection just started, enable the lock
    if (floatLock && floatLockTimer > kFloatLockTimeout) floatLock = false;  // if we are in floatLock, and the locking timer has expired, release the lock
    
    analogWrite(kLEDPin, 0);
  }
}

bool compareDMXArrays() {
  bool updateFlag = false;
  uint8_t current, compare;

  // Compare the array
  for (int c = 0; c < DMX_LENGTH; c++) {
    current = lastDMX[c]; compare = DMXInput[c];
    if (current != compare) { lastDMX[c] = compare; updateFlag = true; }  // if there was a change, note it and trigger update flag
  }

  return updateFlag;
}
