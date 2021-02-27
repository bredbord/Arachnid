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
uint8_t DMXInput[DMX_LENGTH]{0};

constexpr unsigned long kDMXTimeout = 100;  // Millis for considered DMX timeout
constexpr unsigned long kFloatLockTimeout = 2000;  // Millis for Floating Lock timeout
unsigned long kStepperTimeout = 15000;  // Millis for stepper timeout

// Timers
elapsedMillis lastFrameTimer;  // when last DMX frame was received -- for float Lock
elapsedMillis floatLockTimer;  // timer for the float lock
elapsedMillis lastStepperChange;    // when last stepper change

bool floatLock = false;

// The LED pin.
constexpr uint8_t kLEDPin = LED_BUILTIN;


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
bool compareDMXArrays();
void updateStepperDMX();
void runSteppers();
void stopWithError();

void setup() {
  
  // HARD CALL PIN SETUP-------------------------------
  pinMode(kLEDPin, OUTPUT);
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
  if (!floatLock && lastFrameTimer <= kDMXTimeout) { // if we are not in floatLock, and the DMX is valid
    floatLockTimer = 0;  // reset float locking timer
    kStepperTimeout = STEPPER_DMX_TIMEOUT; //set stepper timeout to 3 min
    analogWrite(kLEDPin, 1023);

    // while we have active DMX
    updateStepperDMX();  // update their positions based on DMX
  } 

  // NO DMX---------------------------------
  else {
    // DMX Float Locking
    if (!floatLock && floatLockTimer < kDMXTimeout * 2) floatLock = true; // if the floatLock is not enabled, and the disconection just started, enable the lock
    if (floatLock && floatLockTimer > kFloatLockTimeout) floatLock = false;  // if we are in floatLock, and the locking timer has expired, release the lock

    kStepperTimeout = STEPPER_STANDALONE_TIMEOUT; //set stepper timeout to 30 seconds
    
    analogWrite(kLEDPin, 0);
  }

  // STEPPER UPDATING----------------------
  runSteppers();
}

void runSteppers() {
  int currentPos, newPos;

  // check for any updates to position, update positions, and reset timeout if any occur
  for (int s = 0; s < NUM_STEPPERS; s++) {
    currentPos = stepperPositions[s];
    newPos = newStepperPositions[s];
    
    if (currentPos != newPos) {  // if we have a new position request
      stepperPositions[s] = newPos; // update position index
      lastStepperChange = 0; // reset the timeout
    }
  }

  // If we have not timed out-------------------------------
  if (lastStepperChange < kStepperTimeout) {
    if (!steppersEnabled) { enableSteppers(true); delay(10); }  // if the steppers are not enabled, enable and allow hardware catch up
    
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
    if(DMXInput[DMXOffset + 1] == 0) speedTarget = DEFUALT_MAX_SPEED;
    else speedTarget = map(DMXInput[DMXOffset+1], 1, 255, ABS_STEPPER_MIN_SPEED, ABS_STEPPER_MAX_SPEED);
    stepper[s]->setMaxSpeed(speedTarget);  // map the acceleration from the DMX input

    // Motion Section------------------------------------------------------
    motionTarget = DMXInput[DMXOffset] * MICROSTEPS*4;  // get the new stepper position
    newStepperPositions[s] = motionTarget;
  }
}

void stopWithError() {
  enableSteppers(false);
  
  while (true) {
    analogWrite(kLEDPin, 1023);
    delay(1000);
    analogWrite(kLEDPin, 0);
    delay(1000);
  }
  
}
