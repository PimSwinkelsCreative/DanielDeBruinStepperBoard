#include "serialInterface.h"
#include "settings.h"
#include "stepperControl.h"
#include <Arduino.h>

uint32_t lastSpeedChange = 0;
uint32_t speedChangeInterval = 2000;

float absoluteSpeed = 180; // speed in RPM
float currentSpeed = 0;
float prevSpeed = -absoluteSpeed;

void setup()
{
    delay(3000);
    Serial.begin(115200);
    Serial.println("START SETUP");
    setCpuFrequencyMhz(80);
    loadAllSettings();
    setupStepper();
    setAcceleration(10);
    Serial.println("FINISHED SETUP");
}

void loop()
{
    if (millis() - lastSpeedChange >= speedChangeInterval) {
        lastSpeedChange = millis();
        if (currentSpeed != 0) {
            prevSpeed = currentSpeed;
            currentSpeed = 0;
        } else {
            if (prevSpeed < 0) {
                prevSpeed = currentSpeed;
                currentSpeed = absoluteSpeed;
            } else {
                prevSpeed = currentSpeed;
                currentSpeed = -absoluteSpeed;
            }
        }
        setSpeed(currentSpeed);
    }
    updateStepper();
    updateSerial();
}