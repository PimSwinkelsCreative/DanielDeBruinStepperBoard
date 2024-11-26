#include "settings.h"
#include <Preferences.h>

Preferences settings;

float speed1;
float speed2;


void loadAllSettings()
{
    settings.begin("settings");

    settings.getFloat("speed1", 0);
    // load all settings here like so:
    targetSpeed = settings.getFloat("speed1", 0);

    settings.end();
}

void saveSettings()
{
    settings.begin("settings");
    settings.putFloat("speed",targetSpeed);
    settings.end();
}
