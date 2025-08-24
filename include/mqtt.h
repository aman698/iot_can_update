
#ifndef __MQTT_H
#define __MQTT_H

#include <ArduinoJson.h>
#include "gsm.h"

extern TinyGsm modem;
extern bool remoteImmobilize;

bool mqttConnect();
void sendJsonData(JsonDocument sendJson);

#endif