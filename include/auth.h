
#ifndef __AUTH_H
#define __AUTH_H

#include <ArduinoJson.h>
#include <SparkFun_ATECCX08a_Arduino_Library.h>
#include <Wire.h>

bool ateccWakeup();
void firstAuth();
void secondAuth(uint8_t *challenge);

extern const char *serialNumber;

#endif