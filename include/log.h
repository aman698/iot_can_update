
#ifndef __LOG_H
#define __LOG_H

#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <ESP32Time.h>

#include "gsm.h"
#include "MPU6050_6Axis_MotionApps612.h"

bool startSDSPI();
void writeLogs(ESP32Time rtc, GPSInfo info, float ypr[3], VectorInt16 aaWorld, bool harshAcceleration, bool harshDeceleration, bool rashTurning, bool crashDetection);

#endif