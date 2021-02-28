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
uint8_t DMXInput[DMX_LENGTH]{0};  // future buffering

constexpr unsigned long kDMXTimeout = 100;  // Millis for considered DMX timeout
constexpr unsigned long kFloatLockTimeout = 2000;  // Millis for Floating Lock timeout
unsigned long kStepperTimeout;  // Millis for stepper timeout

// Timers
elapsedMillis lastFrameTimer;  // when last DMX frame was received -- for float Lock
elapsedMillis floatLockTimer;  // timer for the float lock
elapsedMillis lastStepperChange;    // when last stepper change
elapsedMillis lastDMXChange;  // when the DMX last changed

bool floatLock = false;

// The LED pin.
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

// PROTOTYPES-----------------------------------------

bool homeFixtures(byte, byte, int);
bool allHome();
bool compareDMXArrays();
void updateStepperDMX();
bool updateDMX();
void runSteppers();
void stopWithError();

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

  // DMX SETUP-----------------------------------------
  dmxRx.begin();
  lastFrameTimer = kDMXTimeout;


  // Hardware Calibration------------------------------
  if (!homeFixtures(1,1, 10000)) stopWithError();
}

void loop() {
  int read = dmxRx.readPacket(DMXInput, DMX_START, DMX_START + DMX_LENGTH);
  if (read > 0) lastFrameTimer = 0;  // some DMX data received

  // DMX------------------------------------
  if (!floatLock && lastFrameTimer <= kDMXTimeout) { // if we are not in floatLock, and the DMX connection is valid
    floatLockTimer = 0;  // reset float locking timer
    kStepperTimeout = STEPPER_DMX_TIMEOUT; //set stepper timeout to 30 seconds

    if (updateDMX()) { lastDMXChange = 0; statusLEDBrightness = 127; }  // if we see a DMX change, reset the counter
    statusLEDBrightness = LED_DMX_RX_ACTIVE;
    
    if (lastDMXChange < kStepperTimeout - MOTION_TIMEOUT_DURATION) updateStepperDMX();  // if there is new DMX within the pre-time-out, update based on DMX
    else { statusLEDBrightness = LED_DMX_RX_SLEEP; }
    // otherwise, we are't seeing any new data from lighting software, so ignore DMX, move to nearest holding lock, disable to save power, and hold there until new DMX

  } 

  // NO DMX---------------------------------
  else {
    // DMX Float Locking
    if (!floatLock && floatLockTimer < kDMXTimeout * 2) floatLock = true; // if the floatLock is not enabled, and the disconection just started, enable the lock
    if (floatLock && floatLockTimer > kFloatLockTimeout) floatLock = false;  // if we are in floatLock, and the locking timer has expired, release the lock

    statusLEDBrightness = LED_STDBY;
    analogWrite(statusLEDPin, statusLEDBrightness);
    
    if (allHome()) kStepperTimeout = 500; //if everything is at its endstops, timeout doesn't need to take forever.
    else kStepperTimeout = STEPPER_STANDALONE_TIMEOUT; //set stepper timeout to 10 seconds
    
  }

  analogWrite(statusLEDPin, statusLEDBrightness);

  // STEPPER UPDATING----------------------
  runSteppers();
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

// This function will move all fixtures to their endstops, and reset stepper positions if succesful. Returns true for succesful homing, false if not.
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

bool allHome() {
  for (byte s = 0; s <= NUM_STEPPERS; s++) {  // check each stepper
    if (!digitalRead(limitPins[s]) == LOW) return false;  // for triggered limit switch
  }
  return true;
}

void stopWithError() {
  enableSteppers(false);
  
  while (true) {
    analogWrite(statusLEDPin, 1023);
    delay(1000);
    analogWrite(statusLEDPin, 0);
    delay(1000);
  }
  
}
