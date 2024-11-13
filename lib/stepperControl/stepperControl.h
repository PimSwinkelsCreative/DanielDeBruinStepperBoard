#pragma once

#include <Arduino.h>

#define FORWARD HIGH
#define BACKWARD LOW

enum stepperMode{
    position,
    constantSpeed,
    stationary
};

void setupStepper();

bool setMicrosteps(uint16_t _microSteps);

void enableStepper(bool enable);

//sets the speed in RPM
void setSpeed(float _speed);

void updateSpeed();

//sets the acceleratrion in RPM/minute (or rotations/second/second)
void setAcceleration(float accel);


void updateStepper();