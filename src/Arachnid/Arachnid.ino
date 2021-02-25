/* Arachnid -- Main Driver
 * Larson Rivera (a.k.a bredbord)
 * Last Modified: 11/14/2020
 * Version 3.0
*/

// Libraries
#include <OctoWS2811.h>
#include <TeensyDMX.h>
#include <AccelStepper.h>
#include <Bounce2.h>
#include "config.h"

// HARDWARE SETUP==============================================================

// DMX -----------------------------------------------
namespace teensydmx = ::qindesign::teensydmx;
teensydmx::Receiver dmx{Serial1};
uint8_t DMXData[DMX_LENGTH]{0};
constexpr short kTimeout = 500;

// Steppers-------------------------------------------
AccelStepper M1(1,10,9), M2(1,24,12), M3(1,27,26), M4(1,30,29), M5(1,23,22), M6(1,18,17), M7(1,39,38), M8(1,36,35);
AccelStepper *stepper[NUM_STEPPERS] = {&M1, &M2, &M3, &M4, &M5, &M6, &M7, &M8};

//int stepperData[NUM_STEPPERS][OPERATIONS_PER_STEPPER];
bool steppersEnabled = false;

// Limit Switches-------------------------------------
int limitPins[NUM_SWITCHES] = {11, 25, 28, 31, 19, 40, 37, 34};
Bounce * buttons = new Bounce[NUM_SWITCHES];

// LEDS-----------------------------------------------
DMAMEM int displayMemory[LEDS_PER_FIXTURE*6];
int drawingMemory[LEDS_PER_FIXTURE*6];
const int config = WS2811_GRB | WS2811_800kHz;
OctoWS2811 leds(LEDS_PER_FIXTURE, displayMemory, drawingMemory, config);
unsigned long lastTime = 0;

// Sleep Modes----------------------------------------
bool stdby = false;


// Prototypes=================================================================
void updateSteppers();
void updateLights();
void LEDColor(byte, byte, byte, int);
void LEDColor(byte, byte, byte);
void homeFixtures();
void enableSteppers(bool);
bool standBy(bool); 


// Setup===================================================================
void setup() {
  // Steppers
  for (short i = 0; i < NUM_STEPPERS; i++) { stepper[i]->setMaxSpeed(DEFUALT_MAX_SPEED); stepper[i]->setAcceleration(DEFUALT_MAX_ACCELERATION); }  // Set default speed and accel
  for (short i = 0; i < NUM_SWITCHES; i++) { buttons[i].attach(limitPins[i] , INPUT_PULLUP); buttons[i].interval(10); }
  
  pinMode(STEPPER_SLEEP_PIN, OUTPUT);
  pinMode(STEPPER_ENABLE_PIN, OUTPUT);
  pinMode(13, OUTPUT);
  
  // LEDS
  leds.begin();
  leds.show();

  // Home
  LEDRaiseWhite(1000);
  enableSteppers(true);
  //homeFixtures();

  // DMX
  dmx.begin();
}


// MAIN=============================================================
//==================================================================
void loop() {
  dmx.readPacket(DMXData, DMX_START, DMX_LENGTH);
  if (millis() - dmx.lastPacketTimestamp() < kTimeout) {
    //if (standBy(false)) enableSteppers(true);
    digitalWrite(13, HIGH);
    
    updateLights();
    updateSteppers();
  } else {
    digitalWrite(13, LOW);
    if (standBy(true)) {
      LEDRaiseWhite(1000);
      //homeFixtures();
      //enableSteppers(false);
    } else {
      LEDColor(255,255,255);
    }
  }
}


// Extra Functions====================================================

// Update Stepper positions-------------------------------------------------------------------------------------------------------------------------------------
void updateSteppers() {
  int DMXOffset = 0;
  int target = 0;
  int speedTarget =0;
  
  for (byte fixture = 0; fixture < NUM_FIXTURES; fixture++) {                                                      // for each fixture
    DMXOffset = (fixture * OPERATIONS_PER_FIXTURE);                                                                // calculate the DMX data location
    
    AccelStepper* currentStepper = stepper[fixture];                                                               // get the current Stepper
    if(DMXData[DMXOffset + 1] == 0) speedTarget = DEFUALT_MAX_SPEED;
    else speedTarget = map(DMXData[DMXOffset+1], 1, 255, ABS_STEPPER_MIN_SPEED, ABS_STEPPER_MAX_SPEED);
    
    currentStepper->setMaxSpeed(speedTarget);  // map the acceleration from the DMX input

    target = DMXData[DMXOffset] * MICROSTEPS*4;
    
    if (target > 0) currentStepper->moveTo(target);                   // Make sure the stepper doesn't crash into the fixture
    else currentStepper->moveTo(0);                                   // otherwise, go to zero
  }
  for (byte s = 0; s < NUM_STEPPERS; s++) stepper[s]->run();          // run the steppers
}

// Fixture Homing------------------------------------------------------------------------------------------------------------------------------------------------
void homeFixtures() {
  for (byte s = 0; s < NUM_STEPPERS; s++) stepper[s]->setSpeed(-300*MICROSTEPS);  // set all steppers to run backward

  bool homeFlag = true;  // boolean flag to continue homing. 
  
  while (homeFlag) {
    homeFlag = false;  // assume nothing else needs to be homed
    
    for (byte b = 0; b < NUM_SWITCHES; b++) {  // for every switch
      buttons[b].update();                     // update the state
      
      if (buttons[b].read() == LOW) { if(stepper[b]->currentPosition() != 0) stepper[b]->setCurrentPosition(0); }  // if it's low, set that stepper to zero, if it hasn't been done already
      else { stepper[b]->runSpeed(); homeFlag = true; }                                                          // if it's high, run the stepper and flag the home variable
    }
  }

  for (byte s = 0; s < NUM_STEPPERS; s++) stepper[s]->setSpeed(0);  // set all steppers to zero speed
}

// Function to toggle Stepper Enabling--------------------------------------------------------------------------------------------------------------------------------
void enableSteppers(bool newState) {
  if (!steppersEnabled && newState) { digitalWrite(STEPPER_ENABLE_PIN, LOW); delay(10); digitalWrite(STEPPER_SLEEP_PIN, HIGH); steppersEnabled = true; } // steppers enabled
  if (steppersEnabled && !newState) { digitalWrite(STEPPER_ENABLE_PIN, HIGH); delay(10); digitalWrite(STEPPER_SLEEP_PIN, LOW); steppersEnabled = false; } // steppers disabled
}

// Update Lights------------------------------------------------------------------------------------------------------------------------------------------------------
void updateLights() {
  if (micros() > lastTime + LED_REFRESH) {  // check to see if it's time to refresh
    lastTime = micros();  // record the time for the next update
    
    int ledOffset = 0;  // offset for led array
    int DMXOffset = 0;  // offset for DMX data
    
    for (int fixture = 0; fixture < NUM_FIXTURES; fixture++) {                 // for each fixture
      for (int bar = 0; bar < BARS_PER_FIXTURE; bar++) {                       // for each bar in the fixture
        for (int pixel = 0; pixel < LEDS_PER_BAR; pixel++) {                   // for each pixel in each bar in each fixture
          
          ledOffset = (fixture * LEDS_PER_FIXTURE) + (bar * LEDS_PER_BAR) + pixel;                                                        //calculate led position
          DMXOffset = (fixture * OPERATIONS_PER_FIXTURE) + (STEPPERS_PER_FIXTURE * OPERATIONS_PER_STEPPER) + (bar * OPERATIONS_PER_BAR);  // and dmx data location
          leds.setPixel(ledOffset, DMXData[DMXOffset], DMXData[DMXOffset+1], DMXData[DMXOffset+2]);                                       // update the pixel
        }
      }
    }
    leds.show();  // show the new data
  }
}


// Raise the LEDs to a certain color---------------------------------------------------------------------------------------------------------------------------------
void LEDRaiseWhite(int del) {
  for (byte c = 0; c < 255; c++) {
    for(int j = 0; j <= LEDS_PER_FIXTURE * NUM_FIXTURES; j++) {
      leds.setPixel(j, c, c, c);
    }
  leds.show();
  delayMicroseconds(del);
  }
}

// Set the LEDS to a certain Color----------------------------------------------------------------------------------------------------------------------------------
void LEDColor(byte r, byte g, byte b) {
  for(int j = 0; j <= LEDS_PER_FIXTURE * NUM_FIXTURES; j++) {
      leds.setPixel(j, r, g, b);
    }
  leds.show();
}

// Flip the Sleep Boolean
bool standBy(bool state) {
  if (!stdby && state) { stdby = true; return true; }
  if (stdby && !state) { stdby = false; return true; }
  return false;
}
