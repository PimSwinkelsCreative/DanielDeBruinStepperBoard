#include "stepperControl.h"
#include <Arduino.h>
#include "pinout.h"

//  VALUES TO PLAY WITH
const float rpm = -60;                // speed in rotations per minute. negative number reverses the direction. MAX (+-)600
const float acceleration = 1;         // max acceleration in rotations per second per second. Must always be positive
const uint16_t microsteps = 64;        // microstepping. possible settings: 0,2,4,8,16,32,64
const uint16_t motorCurrent = 10;    // set the coil current in milliAmps. Max 2000
const uint16_t startupCurrent = 10; // set the coil current in milliAmps. Max 2000
const uint16_t startupTime = 5000;    // how many milliseconds the current will be different during ramp up-and down
const bool startOnPower = true;       // if true the driver will start when power is active, if false it will start stationary
// DO NOT ALTER CODE AFTER THIS POINT

//startup variables
bool startupActive = true;
uint32_t lastMotorStart = 0;


//stating:
bool motorActive = false;
bool prevStartButton = false;

void setup()
{

  //general setup
  Serial.begin(115200);
  setCpuFrequencyMhz(80);
  pinMode(MANUAL_CTRL_N, INPUT);

  //confnigure motor driver
  setupStepper(microsteps, motorCurrent);
  setAcceleration(acceleration);
  if (startOnPower) {
    setSpeed(rpm);
    lastMotorStart = millis();
    startupActive = true;
    setDriverCurrent(startupCurrent);
    motorActive = true;
  } else {
    setSpeed(0);
    setDriverCurrent(motorCurrent);
    
  }
}

void loop()
{
      if (startupActive && (millis() - lastMotorStart) > startupTime) {
    startupActive = false;
    setDriverCurrent(motorCurrent);
  }


  if (!digitalRead(MANUAL_CTRL_N)) {
    if (!prevStartButton) {
      if (motorActive) {
        motorActive = false;
        setDriverCurrent(startupCurrent);
        lastMotorStart = millis();
        //stop the motor
        setSpeed(0);
      } else {
        //start the motor
        setSpeed(rpm);
        lastMotorStart = millis();
        startupActive = true;
        setDriverCurrent(startupCurrent);
        motorActive = true;
      }
    }
    prevStartButton = true;
  } else {
    prevStartButton = false;
  }
  updateStepper();
}