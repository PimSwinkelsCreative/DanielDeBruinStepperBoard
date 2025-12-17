#pragma once

#include <Arduino.h>

#define FORWARD HIGH
#define BACKWARD LOW

enum stepperMode {
    position,
    constantSpeed,
    stationary
};

void setupStepper(uint8_t uSteps, uint coilCurrent);

bool setMicrosteps(uint16_t _microSteps);

void enableStepper(bool enable);

// sets the speed in RPM
void setSpeed(float _speed);

void startmotorRotation(float angle);

void updateSpeed();

// sets the acceleratrion in RPM/minute (or rotations/second/second)
void setAcceleration(float accel);

void updateStepper();

bool setDriverCurrent(uint16_t milliAmps);

void setZeroPosition();

void stopStepper();

float getCurrentPosition();

bool movementCompleted();

void setPostionMaxSpeed(float maxSpeed);