#pragma once
#include <Arduino.h>

void updateSerial();

// searches for the string sfind in the string str
// returns true if string found
// returns false if string not found
bool StrContains(char *str, char *sfind);

double doubleFromString(char *str);

int intFromString(char *str);

void getNumberSubstring(char *outputStr, char *inputStr);