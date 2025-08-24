
#include "log.h"

static const byte sck = 5;
static const byte miso = 6;
static const byte mosi = 4;
static const byte cs = 46;

void writeFile(fs::FS &fs, const char *path, const char *message)
{
  File file = fs.open(path, FILE_APPEND);
  if (!file)
  {
    Serial.println("Failed to open file for writing!");
    return;
  }
  if (!file.print(message))
  {
    Serial.println("Failed writing logs!");
  }
  file.close();
}

void writeLogs(ESP32Time rtc, GPSInfo info, float ypr[3], VectorInt16 aaWorld, bool harshAcceleration, bool harshDeceleration, bool rashTurning, bool crashDetection)
{
  String log, filename;
  log += rtc.getTime("[%B %d %Y %H:%M:%S]:");
  log += " Latitude: " + String(info.latitude);
  log += " Longitutde: " + String(info.longitude);
  log += " Altitude: " + String(info.altitude);
  log += " GPS Speed: " + String(info.speed);
  log += " Yaw: " + String(ypr[0] * 180 / M_PI);
  log += " Pitch: " + String(ypr[1] * 180 / M_PI);
  log += " Roll: " + String(ypr[2] * 180 / M_PI);
  log += " Accel_X: " + String(aaWorld.x);
  log += " Accel_Y: " + String(aaWorld.y);
  log += " Accel_Z: " + String(aaWorld.z);
  log += " Harsh Acceleration: " + String(harshAcceleration);
  log += " Harsh Deceleration: " + String(harshDeceleration);
  log += " Rash Turning: " + String(rashTurning);
  log += " Crash Detected: " + String(crashDetection);

  filename += "/" + rtc.getTime("%d-%m-%Y") + ".txt";

  writeFile(SD, filename.c_str(), log.c_str());
}

bool startSDSPI()
{
  SPI.begin(sck, miso, mosi);
  return SD.begin(cs);
}