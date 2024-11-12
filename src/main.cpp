#include "stepperControl.h"
#include <Arduino.h>

uint16_t stepsPerSecond = 500;

uint32_t lastStepTime = 0;
uint32_t stepInterval = 1000 / stepsPerSecond;

void setup()
{
  setCpuFrequencyMhz(80);
    setupStepper();
    setSpeed(1000);
}

void loop()
{

    if (millis() % 4000 < 2000) {
        setSpeed(500);
    } else {
       setSpeed(0);
    }

    updateStepper();
}