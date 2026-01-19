#include "pinout.h"
#include "stepperControl.h"
#include <Arduino.h>

// Required external libraries to run this sketch:
//  AccelStepper library:
//  https://github.com/waspinator/AccelStepper

// TMC stepper library:
// https://github.com/teemuatlut/TMCStepper

// code structure:
// ├─ User config
// ├─ Globals / state
// ├─ Small helpers (speed, buttons)
// ├─ Mode handlers
// │    ├─ handleConstant()
// │    ├─ handleConstantReturn()
// │    ├─ handleManual()
// │    └─ handleManualReturn()
// └─ loop()

// ============================= ENUMS =============================

enum moveMode {
    CONSTANT,
    CONSTANTRETURN,
    MANUAL,
    MANUALRETURN
};

enum speedSetting {
    SPEED1,
    SPEED2
};

// ============================= USER CONFIG =============================

// =============================  VALUES TO PLAY WITH ==================================

// set operating mode:
const moveMode mode = MANUAL; // determines what moving mode is used. Options are: CONSTANT, CONSTANTRETURN, MANUAL, and MANUALRETURN
const bool homingEnabled = true; // Set to true to enable homing to SW1 on startup. if false no homing is required. SW1 is used for homing
const bool startOnPower = false; // if true, the driver will start when power is active, if false it will start stationary

// set movement parameters
const bool useSwitchesForRotationAmount = false; // if this is enabled, the movement amount will be determined by the limit switches. Otherwise it will be based on rotationcount
const float rotationsForward = 1; // how many forwardrotations in one move. Counted in whole rotations. (only relevant when useSwitchesForRotationAmount is false)
const float rotationsBackward = 2; // how many forwardrotations in one move. Counted in whole rotations. (only relevant when useSwitchesForRotationAmount is false)
const uint16_t numberOfCycles = 2; // how many times the motor moves the forward/backward movement. Only relevant in MANUAL and MANUALRETURN mode

// set speed and acceleration parameters:
const float rpm_1 = 12.5; // Default speed in rotations per minute. Negative number reverses the direction. MAX (+-)600
const float rpm_2 = 60; // secondary speed in rpm. Toggled by speed button
const float homingSpeed = -10; // homing speed in rpm. Low speed is advised to minimize overshoot. Only relevant when homing is enabled
const float acceleration = 30; // max acceleration in rotations per second per second. Must always be positive

// set hardware config:
const uint16_t microsteps = 16; // microstepping. possible settings: 0,2,4,8,16,32,64. driver internally interpolates everything to 256 steps
const uint16_t motorCurrent = 1000; // set the coil current in milliAmps. Max 2000
// const uint16_t startupCurrent = 100; // set the coil current in milliAmps. Max 2000
// const uint16_t startupTime = 1000; // how many milliseconds the current will be different during ramp up-and down

// ========================= DO NOT ALTER CODE AFTER THIS POINT =========================

// ========================= INTERNAL STATE =========================

// startup variables
bool startupActive = true;
uint32_t lastMotorStart = 0;

// stating:
// bool motorActive = false;
bool motorStartRequired = false;
bool manualButtonFlag = false;
bool updateMotorSpeed = false;

speedSetting currentSpeedSetting = SPEED1;
int currentDirection = 1;

// button stating:
bool prevDirButton = false;
bool prevSpeedButton = false;
bool prevManualButton = false;

// ============================= HELPERS =============================

int getButtonStatus(uint8_t pin)
{
    return !digitalRead(pin); // active low
}

bool buttonEdge(uint8_t pin, bool& prev)
{
    bool pressed = getButtonStatus(pin);
    bool edge = pressed && !prev;
    prev = pressed;
    return edge;
}

inline float selectedRPM()
{
    return (currentSpeedSetting == SPEED1) ? rpm_1 : rpm_2;
}

inline void updateContinuousSpeed(bool active, bool forward = true)
{
    float speed = 0;
    if (active) {
        speed = selectedRPM();
        if (!forward)
            speed = -speed;
        speed *= currentDirection;
    }

    setSpeed(speed);
    lastMotorStart = millis();
    startupActive = true;
    updateMotorSpeed = false;
}

inline void updatePositionSpeed(bool active = true)
{
    if (active) {
        setPostionMaxSpeed(selectedRPM());
    } else {
        setPostionMaxSpeed(0);
    }

    updateMotorSpeed = false;
}

// ============================= MODE HANDLERS =============================

void handleConstant()
{
    static bool movementActive = false;

    if (manualButtonFlag) {
        movementActive = !movementActive;
        updateMotorSpeed = true;
        manualButtonFlag = false;
    }

    if (updateMotorSpeed) {
        updateContinuousSpeed(movementActive);
    }
}

void handleConstantReturn()
{
    static bool movementActive = false;
    static bool movingForward = true;

    // update the manual button:
    if (manualButtonFlag) {
        movementActive = !movementActive;
        updateMotorSpeed = true;
        manualButtonFlag = false;
    }

    if (useSwitchesForRotationAmount) {

        if (getButtonStatus(SW1_N))
            movingForward = true;
        if (getButtonStatus(SW2_N))
            movingForward = false;

        updateContinuousSpeed(movementActive, movingForward);

    } else {

        if (movementCompleted()) {
            if (movementActive) {
                startmotorRotation(
                    (movingForward ? rotationsForward : -rotationsBackward) * currentDirection);
            }
            movingForward = !movingForward;
        }

        if (updateMotorSpeed) {
            updatePositionSpeed(movementActive);
        }
    }
}

void handleManual()
{
    static bool movementStartFlag = false;
    static bool movementActive = false;
    static bool rotationBusy = false;

    // update the manual button:
    if (manualButtonFlag) {
        movementActive = !movementActive;
        updateMotorSpeed = true;
        manualButtonFlag = false;
        if (!rotationBusy) {
            movementStartFlag = true;
        }
    }

    if (useSwitchesForRotationAmount) {

        if (movementStartFlag) {
            movementActive = true;
            movementStartFlag = false;
            updateMotorSpeed = true;
        }

        if (getButtonStatus(SW1_N)) {
            movementActive = false;
            updateMotorSpeed = true;
        }

        if (updateMotorSpeed) {
            updateContinuousSpeed(movementActive);
        }

    } else {

        if (movementCompleted()) {
            if (movementStartFlag) {
                startmotorRotation(rotationsForward * currentDirection);
                movementStartFlag = false;
                movementActive = true;
                rotationBusy = true;
            } else {
                movementActive = false;
                rotationBusy = false;
            }
        }

        if (updateMotorSpeed) {
            updatePositionSpeed(movementActive);
        }
    }

    // if (getButtonStatus(MANUAL_CTRL_N) && !movementActive) {
    //     movementStartFlag = true;
    // }
}

void handleManualReturn()
{
    static bool movementStartFlag = false;
    static bool movementActive = false;
    static bool sequencebusy = false;
    static bool movingForward = true;
    static uint8_t currentCycle = 0;

    static bool prevSw1 = false;
    static bool prevSw2 = false;

    if (useSwitchesForRotationAmount) {

        if (movementStartFlag) {
            movementActive = true;
            movingForward = true;
            currentCycle = 0;
            updateMotorSpeed = true;
            movementStartFlag = false;
        }

        if (buttonEdge(SW1_N, prevSw1)) {
            movingForward = true;
            currentCycle++;
            updateMotorSpeed = true;
            if (currentCycle >= numberOfCycles) {
                movementActive = false;
            }
        }

        if (buttonEdge(SW2_N, prevSw2)) {
            movingForward = false;
            updateMotorSpeed = true;
        }

        if (updateMotorSpeed) {
            updateContinuousSpeed(movementActive, movingForward);
        }

    } else {

        if (movementCompleted()) {
            if (movementStartFlag) {
                currentCycle = 0;
                movementActive = true;
                movingForward = true;
                movementStartFlag = false;
                sequencebusy = true;
            }

            if (movementActive) {
                if (movingForward) {
                    if (currentCycle < numberOfCycles) {
                        updatePositionSpeed();
                        startmotorRotation(rotationsForward * currentDirection);
                        currentCycle++;
                    } else {
                        movementActive = false;
                        sequencebusy = false;
                    }
                } else {
                    startmotorRotation(-rotationsBackward * currentDirection);
                }
                movingForward = !movingForward;
            }
        }

        if (updateMotorSpeed) {
            updatePositionSpeed(movementActive);
        }
    }

    if (manualButtonFlag) {
        if (!sequencebusy) {
            movementStartFlag = true;
        }
        movementActive = !movementActive;
        updateMotorSpeed = true;

        manualButtonFlag = false;
    }
}

// ============================= SETUP / LOOP =============================

void setup()
{
    Serial.begin(115200);
    delay(3000);
    setCpuFrequencyMhz(80);

    pinMode(DIRECTION_N, INPUT);
    pinMode(MANUAL_CTRL_N, INPUT);
    pinMode(SPEED_N, INPUT);
    pinMode(SW1_N, INPUT);
    pinMode(SW2_N, INPUT);

    setupStepper(microsteps, motorCurrent);
    setAcceleration(acceleration);
    setPostionMaxSpeed(rpm_1);

    if (homingEnabled) {
        setSpeed(homingSpeed);
        while (!getButtonStatus(SW1_N)) {
            updateStepper();
        }
    }

    setZeroPosition();

    if (startOnPower) {
        motorStartRequired = true;
    }
}

void loop()
{
    if (buttonEdge(SPEED_N, prevSpeedButton)) {
        Serial.println("Speed button pressed");
        currentSpeedSetting = (currentSpeedSetting == SPEED1) ? SPEED2 : SPEED1;
        updateMotorSpeed = true;
    }

    if (buttonEdge(DIRECTION_N, prevDirButton)) {
        Serial.println("Direction button pressed");
        currentDirection = -currentDirection;
        updateMotorSpeed = true;
    }

    if (buttonEdge(MANUAL_CTRL_N, prevManualButton) || motorStartRequired) {
        Serial.println("Manual button pressed");
        motorStartRequired = false;
        manualButtonFlag = true;
    }

    switch (mode) {
    case CONSTANT:
        handleConstant();
        break;
    case CONSTANTRETURN:
        handleConstantReturn();
        break;
    case MANUAL:
        handleManual();
        break;
    case MANUALRETURN:
        handleManualReturn();
        break;
    }

    updateStepper();
}
