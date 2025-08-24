
#include "auth.h"
#include "mqtt.h"

static ATECCX08A atecc;

void firstAuth()
{
  JsonDocument sendJson;

  atecc.createSignature((uint8_t *)serialNumber, 2);
  sendJson["serialNumber"] = serialNumber;
  sendJson["signature"] = atecc.signature;
  sendJsonData(sendJson);
}

void secondAuth(uint8_t *challenge)
{
  JsonDocument sendJson;

  atecc.createSignature(challenge, 2);
  sendJson["signature"] = atecc.signature;
  sendJsonData(sendJson);
}

bool ateccWakeup()
{
  return atecc.begin(96U, Wire1);
}