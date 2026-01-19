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
const moveMode mode = MANUALRETURN; // determines what moving mode is used. Options are: CONSTANT, CONSTANTRETURN, MANUAL, and MANUALRETURN
const bool homingEnabled = false; // Set to true to enable homing to SW1 on startup. if false no homing is required. SW1 is used for homing
const bool startOnPower = false; // if true, the driver will start when power is active, if false it will start stationary

// set movement parameters
const bool useSwitchesForRotationAmount = true; // if this is enabled, the movement amount will be determined by the limit switches. Otherwise it will be based on rotationcount
const float rotationsForward = 1; // how many forwardrotations in one move. Counted in whole rotations. (only relevant when useSwitchesForRotationAmount is false)
const float rotationsBackward = 2; // how many forwardrotations in one move. Counted in whole rotations. (only relevant when useSwitchesForRotationAmount is false)
const uint16_t numberOfCycles = 2; // how many times the motor moves the forward/backward movement. Only relevant in MANUAL and MANUALRETURN mode

// set speed and acceleration parameters:
const float rpm_1 = 1; // Default speed in rotations per minute. Negative number reverses the direction. MAX (+-)600
const float rpm_2 = 3; // secondary speed in rpm. Toggled by speed button
const float homingSpeed = -10; // homing speed in rpm. Low speed is advised to minimize overshoot. Only relevant when homing is enabled
const float acceleration = 100; // max acceleration in rotations per second per second. Must always be positive

// set hardware config:
const uint16_t microsteps = 64; // microstepping. possible settings: 0,2,4,8,16,32,64. driver internally interpolates everything to 256 steps
const uint16_t motorCurrent = 1000; // set the coil current in milliAmps. Max 2000
// const uint16_t startupCurrent = 100; // set the coil current in milliAmps. Max 2000
// const uint16_t startupTime = 1000; // how many milliseconds the current will be different during ramp up-and down

// ========================= DO NOT ALTER CODE AFTER THIS POINT =========================

// startup variables
bool startupActive = true;
uint32_t lastMotorStart = 0;

// stating:
bool motorActive = false;
bool motorStartRequired = false;
bool manualButtonFlag = false;
    bool updateMotorSpeed = false;

speedSetting currentSpeedSetting = SPEED1;
int currentDirection = 1;

// button stating:
bool prevDirButton = false;
bool prevSpeedButton = false;
bool prevManualButton = false;

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
    setPostionMaxSpeed(rpm_1); // start by default on speed 1

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

    // handle the manual button:
    if (getButtonStatus(MANUAL_CTRL_N) || motorStartRequired) {
        if (!prevManualButton) {
            motorStartRequired = false;
            manualButtonFlag = true;
            prevManualButton = true;
            Serial.println("manualButtonFlag set");
        }
    } else {
        prevManualButton = false;
    }

    switch (mode) {
    case CONSTANT: {
        // hande the manual button:
        // a button press is simulated when a motor start is required by another part of the code
        if (manualButtonFlag) {
            motorStartRequired = false;
            motorActive = !motorActive;
            updateMotorSpeed = true;
            manualButtonFlag = false;
        }

        // update the motor speed if required:
        if (updateMotorSpeed) {
            float targetSpeed = 0;
            if (motorActive) {
                if (currentSpeedSetting == SPEED1) {
                    targetSpeed = rpm_1;
                } else {
                    targetSpeed = rpm_2;
                }
            }
            setSpeed(targetSpeed * currentDirection);
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

            // determine the speed target:
            float targetSpeed = rpm_1;
            if (currentSpeedSetting == SPEED2) {
                targetSpeed = rpm_2;
            }
            if (!movingForward) {
                targetSpeed = -targetSpeed;
            }

            updateMotorSpeed = false; // in this mode the motorspeed gets adjusted automatically, without the need to check this flag

            // set the speed target:
            setSpeed(targetSpeed * currentDirection);
        } else {
            if (movementCompleted()) {
                if (movingForward) {
                    startmotorRotation(rotationsForward * currentDirection);
                } else {
                    startmotorRotation(-rotationsBackward * currentDirection);
                }
                movingForward = !movingForward;
            }

            // update the motor speed when necessary:
            if (updateMotorSpeed) {
                if (currentSpeedSetting == SPEED1) {
                    setPostionMaxSpeed(rpm_1);
                } else {
                    setPostionMaxSpeed(rpm_2);
                }
                updateMotorSpeed = false;
            }
        }

    } break;
    case MANUAL: {
        static bool movementStartFlag = false;
        static bool movementActive = false;

        if (useSwitchesForRotationAmount) {
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

            // handle the "manual" button
            if (movementStartFlag) {
                motorActive = true;
                movementStartFlag = false;
                updateMotorSpeed = true;
            }

            // use SW1 to determine the end of a rotation
            if (getButtonStatus(SW1_N)) {
                motorActive = false;
                updateMotorSpeed = true;
            }

            // update the motor speed if required:
            if (updateMotorSpeed) {
                float targetSpeed = 0;
                if (motorActive) {
                    if (currentSpeedSetting == SPEED1) {
                        targetSpeed = rpm_1;
                    } else {
                        targetSpeed = rpm_2;
                    }
                }
                setSpeed(targetSpeed * currentDirection);
                lastMotorStart = millis();
                startupActive = true;
                // setDriverCurrent(startupCurrent);
            }
        } else {
            if (movementCompleted()) {
                if (movementStartFlag) {
                    // should not be required to update the speed right here as well.. works as temp workaround.
                    if (currentSpeedSetting == SPEED1) {
                        setPostionMaxSpeed(rpm_1);
                    } else {
                        setPostionMaxSpeed(rpm_2);
                    }
                    startmotorRotation(rotationsForward * currentDirection);
                    movementStartFlag = false;
                    movementActive = true;
                } else {
                    movementActive = false;
                }
            }

            // update the motor speed when necessary:
            if (updateMotorSpeed) {
                if (currentSpeedSetting == SPEED1) {
                    setPostionMaxSpeed(rpm_1);
                } else {
                    setPostionMaxSpeed(rpm_2);
                }
                updateMotorSpeed = false;
            }
        }

        // read the "manual" button. Ignore it if a movement is already active.
        if (getButtonStatus(MANUAL_CTRL_N) && !movementActive) {
            movementStartFlag = true;
        }

    } break;
    case MANUALRETURN: {
        static bool movementStartFlag = false;
        static bool movementActive = false;
        static bool movingForward = true;
        static uint8_t currentCycle = 0;

        // button stating:
        static bool prevSw1 = false;
        static bool prevSw2 = false;
        static bool prevManualSw = false;

        if (useSwitchesForRotationAmount) {

            // handle the "manual" button
            if (movementStartFlag) {
                // motorActive = true;
                movementActive = true;
                movementStartFlag = false;
                updateMotorSpeed = true;
                movingForward = true;
                currentCycle = 0;
            }

            // use SW1 to determine the "home" end of a rotation
            if (getButtonStatus(SW1_N)) {
                if (!prevSw1) {
                    prevSw1 = true;
                    movingForward = true;
                    updateMotorSpeed = true;
                    currentCycle++;
                    Serial.println("switching to moving forward");
                    if (currentCycle >= numberOfCycles) {
                        // numbr of cycles is reached. Stop the motor from turning
                        movementActive = false;
                        Serial.println("sequence completed!");
                    }
                }

            } else {
                prevSw1 = false;
            }

            // use SW1 to determine the "far" end of a rotation
            if (getButtonStatus(SW2_N)) {
                if (!prevSw2) {
                    prevSw2 = true;
                    movingForward = false;
                    Serial.println("switching to moving backward");
                    updateMotorSpeed = true;
                }
            } else {
                prevSw2 = false;
            }

            // update the motor speed if required:
            if (updateMotorSpeed) {
                float targetSpeed = 0;
                if (movementActive) {
                    if (currentSpeedSetting == SPEED1) {
                        targetSpeed = rpm_1;
                    } else {
                        targetSpeed = rpm_2;
                    }
                    if (currentDirection < 0) {
                        targetSpeed = -targetSpeed;
                    }
                    if (!movingForward) {
                        targetSpeed = -targetSpeed;
                    }
                }

                setSpeed(targetSpeed);
                lastMotorStart = millis();
                startupActive = true;
                updateMotorSpeed = false;
                // setDriverCurrent(startupCurrent);
            }
        } else {
            if (movementCompleted()) {
                if (movementStartFlag) {
                    // start a new sequence of movements
                    currentCycle = 0;
                    movementActive = true;
                    movingForward = true;
                    movementStartFlag = false;
                }
                if (movementActive) {
                    if (movingForward) {
                        if (currentCycle < numberOfCycles) {
                            Serial.println("trying to start a forward rotation");
                            // should not be required to update the speed right here as well.. works as temp workaround.
                            if (currentSpeedSetting == SPEED1) {
                                setPostionMaxSpeed(rpm_1);
                            } else {
                                setPostionMaxSpeed(rpm_2);
                            }
                            startmotorRotation(rotationsForward * currentDirection);
                            currentCycle++;
                        } else {
                            movementActive = false;
                        }
                    } else {
                        Serial.println("trying to start a backward rotation");
                        startmotorRotation(-rotationsBackward * currentDirection);
                    }
                    movingForward = !movingForward;
                }
            }

            // update the motor speed when necessary:
            if (updateMotorSpeed) {
                Serial.println("1");
                if(movementActive){
                    Serial.println("2");
                if (currentSpeedSetting == SPEED1) {
                    setPostionMaxSpeed(rpm_1);
                    Serial.println("3");
                } else {
                    setPostionMaxSpeed(rpm_2);
                    Serial.println("4");
                }
                } else{
                    stopStepper();
                    Serial.println("5");
                }

                updateMotorSpeed = false;
            }
        }

        // read the "manual" button. Ignore it if a movement is already active.
        if (manualButtonFlag) {
            if (movementActive) {
                movementActive = false;
                updateMotorSpeed = true;
                Serial.println("motor should stop...");
            } else {
                movementStartFlag = true;
            }
            manualButtonFlag = false;
        }
    } break;
    default:
        break;
    }

    // // set the motor current back after a speed change is completed:
    // if (startupActive && (millis() - lastMotorStart) > startupTime) {
    //     startupActive = false;
    //     setDriverCurrent(motorCurrent);
    // }

    updateStepper();
}