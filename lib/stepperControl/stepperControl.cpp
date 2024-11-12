#include "stepperControl.h"
#include "pinout.h"
#include <AccelStepper.h>
#include <TMCStepper.h>

// the tmc2209 stepper driver config control:
HardwareSerial stepperSerial(2);
TMC2209Stepper driver(&stepperSerial, 0.11f, 0);

// accel stepper oject:
AccelStepper stepper(AccelStepper::DRIVER, STEPPER_STEP, STEPPER_DIR);

void setupStepper()
{
    // start the serial communication:
    stepperSerial.begin(9600, SERIAL_8N1, STEPPER_RX, STEPPER_TX);

    // set the pins:
    pinMode(STEPPER_DIR, OUTPUT);
    pinMode(STEPPER_STEP, OUTPUT);
    pinMode(STEPPER_EN, OUTPUT);

    digitalWrite(STEPPER_STEP, LOW);
    setDirection(FORWARD);

    driver.begin();

    // set rms current and microstep
    driver.rms_current(800);
    driver.microsteps(0);

    // enable stealthchop
    driver.pwm_autoscale(true); // Needed for stealthChop
    driver.en_spreadCycle(false); // false = StealthChop / true = SpreadCycle
    driver.shaft(false);

    // initialize the accelStepper library:
    stepper.setMaxSpeed(1000);
    stepper.setAcceleration(1);

    enableStepper(true);
}

void enableStepper(bool enable)
{
    if (enable) {
        digitalWrite(STEPPER_EN, LOW);
    } else {
        digitalWrite(STEPPER_EN, HIGH);
    }
}

bool setDirection(uint8_t direction)
{
    if (direction == FORWARD || direction == BACKWARD) {
        digitalWrite(STEPPER_DIR, direction);
        return true;
    }
    return false;
}

void makeStep()
{
    digitalWrite(STEPPER_STEP, HIGH);
    delayMicroseconds(1);
    digitalWrite(STEPPER_STEP, LOW);
}

void setSpeed(float speed)
{
    stepper.setSpeed(speed);
    stepper.runSpeed();
}

void updateStepper()
{
    stepper.runSpeed();
}
