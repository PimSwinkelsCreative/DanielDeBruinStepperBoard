#pragma once

#include <Arduino.h>

#define FORWARD HIGH
#define BACKWARD LOW

void setupStepper();

bool setDirection(uint8_t direction);

void enableStepper(bool enable);

void makeStep();

void setSpeed(float speed);

void updateStepper();
