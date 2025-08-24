
#include "mqtt.h"
#include "auth.h"
#include "vehiclecontrol.h"

static const char *clientID = "IOT_DEVICE_01";                    // Client ID
static char subscribeTopic[] = "devices/test-device-001/control"; // Sub Topic
static char publishTopic[] = "devices/test-device-001/telemetry"; // Pub Topic
static const char *broker = "13.204.67.195";
static const uint16_t brokerPort = 8883;
static const uint8_t mqttClientID = 0;
bool remoteImmobilize = false;

void sendJsonData(JsonDocument sendJson)
{
  String pubres;
  serializeJson(sendJson, pubres);
  int str_len = pubres.length() + 1;
  char char_array[str_len];
  pubres.toCharArray(char_array, str_len);
  modem.mqtt_publish(0, publishTopic, char_array);
}

void recieveJsonData(const char *topic, const uint8_t *payload, uint32_t len)
{
  String inputString;
  JsonDocument readJson;

  for (int i = 0; i < len; i++)
  {
    inputString += (char)payload[i];
  }

  Serial.println(inputString);
  int jsonBeginAt = inputString.indexOf("{");
  int jsonEndAt = inputString.lastIndexOf("}");
  if (jsonBeginAt != -1 && jsonEndAt != -1)
  {
    inputString = inputString.substring(jsonBeginAt, jsonEndAt + 1);
    deserializeJson(readJson, inputString);
  }

  remoteImmobilize = readJson["lock"];
  if (remoteImmobilize == false)
    setMotorMovementLock(false);

  if (readJson["isAuthActive"] == true)
  {
    Serial.println("Authentication has been started....");
  }
  else if (readJson["isFirstAuthSuccess"] == true)
  {
    const char *sig = readJson["challenge"];
    secondAuth((uint8_t *)sig);
  }
  else if (readJson["isSecondAuthSuccess"] == true)
  {
    Serial.println("Authentication went successfully!");
  }
}

bool mqttConnect()
{
  Serial.print("Connecting to ");
  Serial.println(broker);

  while (!modem.mqtt_connected())
  {
    modem.mqtt_connect(mqttClientID, broker, brokerPort, clientID);
    Serial.println("Trying....");
    delay(500);
  }

  if (modem.mqtt_connected())
  {
    Serial.println("Connected to the server!");
  }
  // Set MQTT processing callback
  modem.mqtt_set_callback(recieveJsonData);
  // Subscribe to topic
  modem.mqtt_subscribe(mqttClientID, subscribeTopic);

  return true;
}