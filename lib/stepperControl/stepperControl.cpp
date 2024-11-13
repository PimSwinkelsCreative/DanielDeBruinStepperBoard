#include "stepperControl.h"
#include "pinout.h"
#include <AccelStepper.h>
#include <TMCStepper.h>

// the tmc2209 stepper driver config control:
HardwareSerial stepperSerial(2);
TMC2209Stepper driver(&stepperSerial, 0.11f, 0);

// accel stepper oject:
AccelStepper stepper(AccelStepper::DRIVER, STEPPER_STEP, STEPPER_DIR);

// parameters:
stepperMode mode = position;
uint16_t stepsPerRevolution = 200;
uint16_t microSteps = 16;
uint16_t microStepsPerRevolution = microSteps * stepsPerRevolution;
float acceleration = 10; // accelerations in rotations per second

// speed variables:
float speed = 0;
float targetSpeed = 0;
uint32_t lastSpeedUpdate = 0;

void setupStepper()
{
    // start the serial communication:
    stepperSerial.begin(9600, SERIAL_8N1, STEPPER_RX, STEPPER_TX);

    // set the pins:
    pinMode(STEPPER_DIR, OUTPUT);
    pinMode(STEPPER_STEP, OUTPUT);
    pinMode(STEPPER_EN, OUTPUT);
    digitalWrite(STEPPER_STEP, LOW);

    enableStepper(false);

    // start driver config:
    driver.begin();

    // set rms current and microstep
    driver.rms_current(800);
    if (!setMicrosteps(0)) {
        Serial.println("ERROR: Could not configure microsteps!");
    }

    // enable stealthchop
    driver.pwm_autoscale(true); // Needed for stealthChop
    driver.en_spreadCycle(false); // false = StealthChop / true = SpreadCycle
    driver.shaft(false);

    // initialize the accelStepper library:
    stepper.setMaxSpeed(10 * microStepsPerRevolution); // limit speed to 10Hz
    setAcceleration(1);

    enableStepper(true);
}

bool setMicrosteps(uint16_t _microSteps)
{
    // check if the number is a power of two and within range:
    if (_microSteps & (_microSteps - 1) != 0 || _microSteps > 256) {
        return false;
    }
    microSteps = _microSteps;
    if (microSteps) {
        microStepsPerRevolution = microSteps * stepsPerRevolution;
    } else {
        microStepsPerRevolution = stepsPerRevolution;
    }
    driver.microsteps(microSteps);
    Serial.println("Microsteps: " + String(microSteps));
    Serial.println("Steps per revolution: " + String(microStepsPerRevolution));
    return true;
}

void enableStepper(bool enable)
{
    if (enable) {
        digitalWrite(STEPPER_EN, LOW);
    } else {
        digitalWrite(STEPPER_EN, HIGH);
    }
}

void setSpeed(float _speed)
{
    mode = constantSpeed;
    targetSpeed = _speed;
    lastSpeedUpdate = micros(); // reset the speed update timer
}

void setAcceleration(float accel)
{
    acceleration = accel;
    accel *= float(microStepsPerRevolution);
    stepper.setAcceleration(accel);
}

void updateStepper()
{
    switch (mode) {
    case constantSpeed:
        // the motor should move at a constant speed.
        // the target should always be set far enough away so that the motor reaches full speed.
        updateSpeed();

        stepper.runSpeed();
        break;

    case stationary:
        if (stepper.currentPosition() == stepper.targetPosition()) {
            stepper.setMaxSpeed(0);
        }
        stepper.run();
        break;
    case position:
        stepper.run();
    default:
        break;
    }
}

void updateSpeed()
{
    if (speed != targetSpeed) {
        // calculate the time since the last update:
        uint32_t now = micros();
        uint32_t interval = now - lastSpeedUpdate;
        lastSpeedUpdate = now;
        float speedToAdd = 60.0 * acceleration * float(interval) / 1000000.0;
        float newSpeed = 0;
        if (speed > targetSpeed) {
            newSpeed = constrain(speed - speedToAdd, targetSpeed, speed);
        } else {
            newSpeed = constrain(speed + speedToAdd, speed, targetSpeed);
        }
        speed = newSpeed;
        stepper.setSpeed(speed * microStepsPerRevolution / 60.0);
    }
}
