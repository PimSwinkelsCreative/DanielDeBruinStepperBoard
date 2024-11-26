#include "serialInterface.h"
#include "settings.h"

const int serialBufferLength = 200;
char serialBuffer[serialBufferLength];
uint8_t serialBufferIndex = 0;

void updateSerial()
{
    while (Serial.available()) {
        char inChar = Serial.read();
        if (inChar != '\n') {
            // just add it to the buffer
            serialBuffer[serialBufferIndex] = inChar;
            serialBufferIndex++;
            if (serialBufferIndex >= serialBufferLength)
                serialBufferIndex = 0; // avoid buffer overflow
        } else {
            serialBuffer[serialBufferIndex] = '\0'; // place a null terminator on the char array
            // convert to lower case if upper case
            for (int i = 0; i < serialBufferIndex; i++) {
                if (serialBuffer[i] >= 'A' && serialBuffer[i] <= 'Z') {
                    serialBuffer[i] += ('a' - 'A');
                }
            }

            // parse the string:
            if (strcmp(serialBuffer, "save") == 0) {
                // save the current settings
                saveSettings();
                Serial.println(F("Settings are saved"));
            } else if (StrContains(serialBuffer, "speed")) {
                double setpoint = doubleFromString(serialBuffer);
                if (setpoint >= -30 && setpoint <= 30) {
                    targetSpeed = setpoint;
                    Serial.print(F("Speed set to "));
                    Serial.print(targetSpeed);
                    Serial.println(" RPM");
                } else {
                    Serial.println(
                        F("Speed setting is not a valid input, try numbers between -30.0 "
                          "and "
                          "30.0"));
                }
            } else {
                Serial.println(
                    F("Command not recognised. type \"help\" to get a list of all "
                      "commands"));
            }

            // reset the input buffer
            serialBufferIndex = 0;
        }
    }
}

// searches for the string sfind in the string str
// returns true if string found
// returns false if string not found
bool StrContains(char* str, char* sfind)
{
    char found = 0;
    char index = 0;
    char len;
    len = strlen(str);
    if (strlen(sfind) > len) {
        return 0;
    }
    while (index < len) {
        if (str[index] == sfind[found]) {
            found++;
            if (strlen(sfind) == found) {
                return true;
            }
        } else {
            found = 0;
        }
        index++;
    }
    return false;
}

int intFromString(char* str)
{
    char numberString[strlen(str) + 1];
    getNumberSubstring(numberString, str);
    if (strlen(numberString)) {
        return atoi(numberString);
    } else {
        return 0xFFFF; // very low negative number
    }
}

void getNumberSubstring(char* outputStr, char* inputStr)
{
    int size = strlen(inputStr);
    // search for a number
    int i = 0;
    while ((inputStr[i] < '0' || inputStr[i] > '9') && inputStr[i] != '-' && i < size) {
        i++;
    }
    // copy the number into a substring
    char numberString[strlen(inputStr) + 1];
    int j = 0;
    while (((inputStr[i] >= '0' && inputStr[i] <= '9') || inputStr[i] == '.' || inputStr[i] == '-') && i < size) {
        numberString[j] = inputStr[i];
        i++;
        j++;
    }
    numberString[j] = '\0'; // ad a null terminator at the end of the string
    strcpy(outputStr, numberString);
}

double doubleFromString(char *str) {
  char numberString[strlen(str) + 1];
  getNumberSubstring(numberString, str);
  if (strlen(numberString)) {
    double number = atof(numberString);
    return number;
  } else {
    return -1000000;
  }
}