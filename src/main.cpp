#include "pinout.h"
#include "stepperControl.h"
#include <Arduino.h>

enum moveMode { CONSTANT,
    CONSTANTRETURN,
    MANUAL,
    MANUALRETURN
};

enum speedSetting { SPEED1,
    SPEED2 };

// =============================  VALUES TO PLAY WITH ==================================

// set operating mode:
const moveMode mode = CONSTANTRETURN; // determines whether the unit functions in a constant moving mode, or that it moves whenever pressed
const bool homingEnabled = true; // Set to true to enable homing to SW1 on startup. if false no homing is required
const bool startOnPower = true; // if true the driver will start when power is active, if false it will start stationary

// set movement parameters
const bool useSwitchesForRotationAmount = false; // if this is enabled, the movement amont will be determined by the limit switches. Otherwise it will be based on rotationcount
const float rotationsForward = 1; // how many forwardrotations in one move. Counted in whole rotations. (only relevant when useSwitchesForRotationAmount is false)
const float rotationsBackward = 1; // how many forwardrotations in one move. Counted in whole rotations. (only relevant when useSwitchesForRotationAmount is false)
const uint16_t numberOfCycles = 3; // how many times the motor moves the forward/backward movement. Only relevant in MANUAL and MANUALRETURN mode

// set speed and acceleration parameters:
const float rpm_1 = 60; // Default speed in rotations per minute. Negative number reverses the direction. MAX (+-)600
const float rpm_2 = 120; // secondary speed in rpm. Toggled by speed button
const float homingSpeed = -10; // homing speed in rpm. Low speed is advised to minimize overshoot. Only relevant when homing is enabled
const float acceleration = 100; // max acceleration in rotations per second per second. Must always be positive

// set hardware config:
const uint16_t microsteps = 64; // microstepping. possible settings: 0,2,4,8,16,32,64
const uint16_t motorCurrent = 500; // set the coil current in milliAmps. Max 2000
// const uint16_t startupCurrent = 100; // set the coil current in milliAmps. Max 2000
// const uint16_t startupTime = 1000; // how many milliseconds the current will be different during ramp up-and down

// ========================= DO NOT ALTER CODE AFTER THIS POINT =========================

// startup variables
bool startupActive = true;
uint32_t lastMotorStart = 0;

// stating:
bool motorActive = false;
bool motorStartRequired = false;

speedSetting currentSpeedSetting = SPEED1;
int currentDirection = 1;
int currentCycle = 0;

// button stating:
bool prevStartButton = false;
bool prevDirButton = false;
bool prevSpeedButton = false;

int getButtonStatus(uint8_t buttonPin)
{
    // all buttons are low active
    return !digitalRead(buttonPin);
}

void setup()
{
    // general setup
    Serial.begin(115200);
    delay(3000);
    Serial.println("Start Setup");
    setCpuFrequencyMhz(80);

    // setup the i/o pins:
    pinMode(DIRECTION_N, INPUT);
    pinMode(MANUAL_CTRL_N, INPUT);
    pinMode(SPEED_N, INPUT);
    pinMode(SW1_N, INPUT);
    pinMode(SW2_N, INPUT);

    // configure motor driver
    setupStepper(microsteps, motorCurrent);
    setAcceleration(acceleration);
    setPostionMaxSpeed(rpm_1); // sytart by defaut on speed 1

    // home if required:
    if (homingEnabled) {
        Serial.println("Start homing...");
        setSpeed(homingSpeed);
        while (!getButtonStatus(SW1_N)) {
            updateStepper();
        }
        Serial.println("homing Finished!");
    }
    setZeroPosition();

    // set the active start:
    if (startOnPower) {
        motorStartRequired = true;
    }

    Serial.println("Setup completed!");
}

void loop()
{
    // handle the speed button:
    bool updateMotorSpeed = false;
    if (getButtonStatus(SPEED_N)) {
        if (!prevSpeedButton) {
            if (currentSpeedSetting == SPEED1) {
                currentSpeedSetting = SPEED2;
            } else {
                currentSpeedSetting = SPEED1;
            }
            updateMotorSpeed = true;
        }
        prevSpeedButton = true;
    } else {
        prevSpeedButton = false;
    }

    switch (mode) {
    case CONSTANT: {
        // hande the manual button:
        // a button press is simulated when a motor start is required by another part of the code
        if (getButtonStatus(MANUAL_CTRL_N) || motorStartRequired) {
            if (!prevStartButton) {
                motorStartRequired = false;
                motorActive = !motorActive;
                updateMotorSpeed = true;
            }
            prevStartButton = true;

        } else {
            prevStartButton = false;
        }

        // handle the direction button:
        if (getButtonStatus(DIRECTION_N)) {
            if (!prevDirButton) {
                currentDirection = -currentDirection;
                updateMotorSpeed = true;
            }
            prevDirButton = true;

        } else {
            prevDirButton = false;
        }

        // update the motor speed if required:
        if (updateMotorSpeed) {
            float targetSpeed = 0;
            if (motorActive) {
                if (currentSpeedSetting == SPEED1) {
                    targetSpeed = rpm_1;
                    Serial.println("setting speed 1");
                } else {
                    targetSpeed = rpm_2;
                    Serial.println("setting speed 2");
                }
                if (currentDirection < 0) {
                    targetSpeed = -targetSpeed;
                }
            }
            setSpeed(targetSpeed);
            lastMotorStart = millis();
            startupActive = true;
            // setDriverCurrent(startupCurrent);
        }
    } break;
    case CONSTANTRETURN: {
        static bool movingForward = true;
        if (useSwitchesForRotationAmount) {
            // check what direction the motor should turn in:
            if (getButtonStatus(SW1_N)) {
                movingForward = true;
            }
            if (getButtonStatus(SW2_N)) {
                movingForward = false;
            }

            // deternmine the speed target:
            float targetSpeed = rpm_1;
            if (currentSpeedSetting == SPEED2) {
                targetSpeed = rpm_2;
            }
            if (!movingForward) {
                targetSpeed = -targetSpeed;
            }

            // set the speed target:
            setSpeed(targetSpeed);
        } else {
            if (movementCompleted()) {
                if (movingForward) {
                    startmotorRotation(rotationsForward);
                } else {
                    startmotorRotation(-rotationsBackward);
                }
                movingForward = !movingForward;
            }
        }
    } break;
    case MANUAL: {
        static bool startButtonPressed = false;
        if (startButtonPressed) {
            if (movementCompleted()) {
                startmotorRotation(rotationsForward);
            }
            startButtonPressed = false; // flag will always be cleared to avoid constantly starting a rotation

        } else if (getButtonStatus(MANUAL_CTRL_N)) {
            startButtonPressed = true;
        }
    } break;
    case MANUALRETURN: {
        static bool movingForward = true;
        if (motorActive) {
            if (movementCompleted()) {
                if (currentCycle < numberOfCycles) {
                    if (movingForward) {
                        startmotorRotation(rotationsForward);
                    } else {
                        startmotorRotation(-rotationsBackward);
                        currentCycle++;
                    }
                    movingForward = !movingForward;
                } else {
                    motorActive = false;
                    currentCycle = 0;
                    movingForward = true;
                }
            }
        } else if (getButtonStatus(MANUAL_CTRL_N)) {
            motorActive = true;
        }
    } break;
    default:
        break;
    }

    // // set the motor current back after a peed change is completed:
    // if (startupActive && (millis() - lastMotorStart) > startupTime) {
    //     startupActive = false;
    //     setDriverCurrent(motorCurrent);
    // }

    updateStepper();
}