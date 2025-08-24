
#include "main.h"
#include "mqtt.h"
#include "auth.h"
#include "log.h"
#include "vehiclecontrol.h"
#include "ble.h"
#include "can.h"

static const char apn[] = "airtelgprs.com";
static const char gprsUser[] = "";
static const char gprsPass[] = "";

static const byte RXPin = 17, TXPin = 18;
static const uint32_t modemBaud = 115200;
static const byte INTERRUPT_PIN = 10;

/*---MPU6050 Control/Status Variables---*/
// static Adafruit_MPU6050 mpu;
// static sensors_event_t a, g, temp;

MPU6050 mpu(0x68, &Wire1);

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 gy;      // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
static bool harshAcceleration = false;
static bool harshDeceleration = false;
static bool rashTurning = false;
static bool crashDetection = false;
static byte turnCount = 0;

void dmpDataReady()
{
}

static unsigned long millisSend = 0;
static unsigned long millisLog = 0;
static unsigned long millisTime = 0;
static unsigned long millisIMU = 0;
static unsigned long millisBattery = 0;
static unsigned long millisGSM = 0;
static unsigned long millisCheck = 0;
static unsigned long millisRPM = 0;

static const long sendInterval = 5000;
static const long logInterval = 10000;
static const long timeInterval = 30000000;
static const long IMUInterval = 500;
static const long batteryInterval = 3000;
static const long GSMInterval = 2500;
static const long checkInterval = 5000;
static const long rpmInterval = 1500;

/* LED */
static const byte redPin = 41;
static const byte greenPin = 40;
static const byte bluePin = 39;

/* Battery */
static uint16_t internBatVolt = 0;
static float internBatteryVolt = 0;

static const byte adcPin = 12;
static const float V_REF = 3.3;                        // Analog reference voltage (e.g., 5V or 3.3V)
static const float R_BITS = 12.0;                      // ADC resolution (bits)
static const float ADC_STEPS = (1 << int(R_BITS)) - 1; // Number of steps (2^R_BITS - 1)
static float externBatteryVolt = 0;

/* SPI */

/* GSM */
#define TOTAL_GSM_DETAILS 6

static GPSInfo info;
TinyGsm modem(Serial1);

enum gsmDetailsType
{
  SYSTEM_MODE = 0,
  UNUSED = 1,
  MCC = 2,
  MNC = 3,
  AC = 4,
  CELLID = 5,
};
static int signalQuality = 0;
static unsigned long elapsedFixTime = 0;
static char gsmDetails[TOTAL_GSM_DETAILS][32];
long double gpsOdometer = 0;

/* Time */
static ESP32Time rtc;

/* RPM */
static const byte rpmPin = 34;
static volatile unsigned int rpm = 0;
static long double odometer = 0;

#define PCNT_UNIT PCNT_UNIT_0

portMUX_TYPE pcntMux = portMUX_INITIALIZER_UNLOCKED;

/* IOT Module */
static const char *vendorID = "MZ";
static const char *hardwareNumber = "MZ9303838";
static const char *firmwareVersion = "v1.0.0";
const char *serialNumber = "Db6yR81viLgger5micwKWqaTXOfaQiSH";
static const char *vehicleNumber = "1HGBH41JXMN109186";

void setupPCNT()
{
  pcnt_config_t pcnt_config = {
      .pulse_gpio_num = rpmPin,
      .ctrl_gpio_num = PCNT_PIN_NOT_USED,
      .lctrl_mode = PCNT_MODE_KEEP,
      .hctrl_mode = PCNT_MODE_KEEP,
      .pos_mode = PCNT_COUNT_INC,
      .neg_mode = PCNT_COUNT_DIS,
      .counter_h_lim = 10000,
      .counter_l_lim = 0,
      .unit = PCNT_UNIT,
      .channel = PCNT_CHANNEL_0};

  pcnt_unit_config(&pcnt_config);
  pcnt_counter_pause(PCNT_UNIT);
  pcnt_counter_clear(PCNT_UNIT);
  pcnt_counter_resume(PCNT_UNIT);
  pcnt_set_filter_value(PCNT_UNIT, 1000); // 1000 APB cycles = 12.5 us (at 80 MHz)
  pcnt_filter_enable(PCNT_UNIT);
}
// Timer callback (every 1 sec)
void periodicTimerCallback(void *arg)
{
  int16_t count = 0;

  portENTER_CRITICAL_ISR(&pcntMux);
  pcnt_get_counter_value(PCNT_UNIT, &count);
  pcnt_counter_clear(PCNT_UNIT);
  portEXIT_CRITICAL_ISR(&pcntMux);

  rpm = (count * 60) / 26;
}

void setColor(int redValue, int greenValue, int blueValue)
{
  analogWrite(redPin, redValue);
  analogWrite(greenPin, greenValue);
  analogWrite(bluePin, blueValue);
}

void syncTime()
{
  int year = 0;
  int month = 0;
  int day = 0;
  int hour = 0;
  int min = 0;
  int sec = 0;
  float timezone = 0;
  if (modem.getNetworkTime(&year, &month, &day, &hour, &min, &sec, &timezone))
  {
    rtc.offset = timezone * 3600;
    rtc.setTime(sec, min, hour, day, month, year);
    Serial.print("Time: ");
    Serial.println(rtc.getTime("%A, %B %d %Y %H:%M:%S"));
  }
}

void setup()
{
  esp_log_level_set("*", ESP_LOG_NONE);
  setColor(0, 0, 0);
  initVehicleControlPins();
  setIgnitionLock(true);
  setMotorMovementLock(true);

  pinMode(rpmPin, INPUT_PULLUP);
  pinMode(adcPin, INPUT);
  analogReadResolution(12); // 9:0-511 10:0-1023 12:0-4096

  setupPCNT();

  // Setup 1-second periodic timer
  const esp_timer_create_args_t timer_args = {
      .callback = &periodicTimerCallback,
      .arg = nullptr,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "1s_pc_timer"};

  esp_timer_handle_t timer;
  esp_timer_create(&timer_args, &timer);
  esp_timer_start_periodic(timer, 1000000); // 1,000,000 us = 1 sec

  Serial.begin(115200);
  Serial1.begin(modemBaud, SERIAL_8N1, RXPin, TXPin);

  /*Initialize devices*/
  setColor(255, 255, 0);
  Wire.begin(3, 2);
  Wire1.begin(9, 8, 400000);
  CAN_Init();

  if (esp_sleep_get_wakeup_cause() == 2)
  {
    if (!startSDSPI())
    {
      Serial.println(F("Card mount failed!"));
      setColor(255, 0, 0);
      while (true)
        ;
    }
    setColor(0, 255, 0);
    return;
  }

  delay(3000);
  Serial.println(F("Initializing I2C devices..."));

  /* Initialize IMU */ 
  mpu.initialize();
  pinMode(10, INPUT);

  if (!mpu.testConnection())
  {
    setColor(0, 255, 0);
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(64);
  mpu.setYGyroOffset(-21);
  mpu.setZGyroOffset(-1);
  mpu.setXAccelOffset(-1843);
  mpu.setYAccelOffset(-969);
  mpu.setZAccelOffset(3210);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(10);
    mpu.CalibrateGyro(10);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    mpu.setFIFOTimeout(50000);
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    setColor(255, 0, 0);
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
    while (1)
    {
      delay(10);
    }
  }
  mpu.setTempSensorEnabled(true);

  Serial.println(F("Initializing SPI devices..."));

  if (!startSDSPI())
  {
    Serial.println(F("Card mount failed!"));
    setColor(255, 0, 0);
    while (true)
      ;
  }

  Serial.println("Initializing modem...");
  int retry = 0;
  modem.restart();
  while (!modem.testAT(1000))
  {
    Serial.println(".");
    if (retry++ > 10)
    {
      retry = 0;
    }
  }
  // Check if SIM card is online
  SimStatus sim = SIM_ERROR;
  while (sim != SIM_READY)
  {
    sim = modem.getSimStatus();
    switch (sim)
    {
    case SIM_READY:
      Serial.println("SIM card online");
      break;
    case SIM_LOCKED:
      Serial.println("The SIM card is locked. Please unlock the SIM card first.");
      // const char *SIMCARD_PIN_CODE = "123456";
      // modem.simUnlock(SIMCARD_PIN_CODE);
      break;
    default:
      break;
    }
    delay(1000);
  }

  Serial.printf("Set network apn : %s\n", apn);
  modem.sendAT(GF("+CGDCONT=1,\"IP\",\""), apn, "\"");
  if (modem.waitResponse() != 1)
  {
    Serial.println("Set network apn error!");
  }

  // Check network registration status and network signal status
  int16_t sq;
  Serial.print("Wait for the modem to register with the network.");
  RegStatus status = REG_NO_RESULT;
  while (status == REG_NO_RESULT || status == REG_SEARCHING || status == REG_UNREGISTERED)
  {
    status = modem.getRegistrationStatus();
    switch (status)
    {
    case REG_UNREGISTERED:
    case REG_SEARCHING:
      sq = modem.getSignalQuality();
      Serial.printf("[%lu] Signal Quality:%d\n", millis() / 1000, sq);
      delay(1000);
      break;
    case REG_DENIED:
      Serial.println("Network registration was rejected, please check if the APN is correct");
      return;
    case REG_OK_HOME:
      Serial.println("Online registration successful");
      break;
    case REG_OK_ROAMING:
      Serial.println("Network registration successful, currently in roaming mode");
      break;
    default:
      Serial.printf("Registration Status:%d\n", status);
      delay(1000);
      break;
    }
  }
  Serial.println();

  Serial.printf("Registration Status:%d\n", status);
  delay(1000);

  String ueInfo;
  if (modem.getSystemInformation(ueInfo))
  {
    Serial.print("Inquiring UE system information:");
    Serial.println(ueInfo);
  }

  if (!modem.setNetworkActive())
  {
    Serial.println("Enable network failed!");
  }

  delay(3000);

  String ipAddress = modem.getLocalIP();
  Serial.print("Network IP:");
  Serial.println(ipAddress);

  Serial.println("Enabling GPS/GNSS/GLONASS");
  modem.enableGPS();

  Serial.println("Syncing time...");
  syncTime();

  Serial.println("Starting Bluetooth...");
  BLEInit();

  modem.mqtt_begin(true);

  mqttConnect();

  esp_sleep_enable_ext0_wakeup(GPIO_NUM_11, 1);
  // esp_deep_sleep_start();

  Serial.println("Starting application");
}

void loop()
{
  if (CAN_Read()) {
    CAN_PrintFrame();
  }
  if (BLELoop())
  {
    setIgnitionLock(true);
    setMotorMovementLock(true);
    setColor(255, 255, 255);
  }

  else
  {
    modem.mqtt_handle(1000);

    if ((millis() - millisRPM) > rpmInterval)
    {
      millisRPM = millis();

      odometer += rpm * M_PI * 0.0004064 * 0.025; // Reading in KM

      if (rpm == 0 && remoteImmobilize)
        setMotorMovementLock(true);
    }

    if ((millis() - millisIMU) > IMUInterval)
    {
      millisIMU = millis();
/*
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
      {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
      }
*/
      if ((ypr[1] * 180 / M_PI) > 20 || (ypr[1] * 180 / M_PI) < -20)
        rashTurning = true;

      if ((ypr[1] * 180 / M_PI) > 30 || (ypr[1] * 180 / M_PI) < -30)
        crashDetection = true;

      if ((aaWorld.x / 16384.00) * 9.8 > 4)
        harshDeceleration = true;

      if ((aaWorld.x / 16384.00) * 9.8 > 3)
      {
        turnCount++;
        if (turnCount > 4)
          harshAcceleration = true;
      }
    }

    if ((millis() - millisGSM) > GSMInterval)
    {
      millisGSM = millis();

      String gsmData = modem.getOtherDetails();
      int len = gsmData.length() + 1;
      char tempArr[len];

      gsmData.toCharArray(tempArr, len);
      char *ptr = strtok(tempArr, ",");
      int i = 0;
      for (int i = 0; i < TOTAL_GSM_DETAILS; i++)
      {
        strcpy(gsmDetails[i], ptr);
        ptr = strtok(NULL, ",");
      }
      signalQuality = modem.getSignalQuality();

      modem.getGPS_Ex(info);

      if (info.isFix == 2 || info.isFix == 3)
      {
        elapsedFixTime = 0;
        gpsOdometer += info.speed * 1.852 * 0.000694444;
      }
      else
      {
        info.isFix = 0;
        elapsedFixTime += 2.5;
      }
    }

    if ((millis() - millisBattery) > batteryInterval)
    {
      millisBattery = millis();

      Wire.beginTransmission(MAX17048_I2C_ADDRESS);
      Wire.write(0x02);
      Wire.endTransmission();
      Wire.requestFrom(MAX17048_I2C_ADDRESS, 2);
      internBatVolt = (Wire.read() << 8) | Wire.read();
      internBatteryVolt = (float)internBatVolt * 78.125 / 1000000;
    }

    if ((millis() - millisCheck) > checkInterval)
    {
      millisCheck = millis();

      if (!modem.mqtt_connected())
      {
        setColor(255, 255, 0);
        mqttConnect();
      }
      else
      {
        setColor(0, 255, 0);
      }

      if (modem.isEnableGPS() == false)
      {
        modem.enableGPS();
      }
    }

    if ((millis() - millisSend) > sendInterval)
    {
      millisSend = millis();

      JsonDocument sendJson;

      sendJson["Battery"]["InternBatteryVolt"] = internBatteryVolt;
      if (analogRead(adcPin) < 1100)
        sendJson["Battery"]["ExternBatteryVolt"] = 0;
      else
        sendJson["Battery"]["ExternBatteryVolt"] = 48.5;

      sendJson["GSM"]["ICCID"] = modem.getSimCCID();
      sendJson["GSM"]["IMEI"] = modem.getIMEI();
      sendJson["GSM"]["IMSI"] = modem.getIMSI();
      sendJson["GSM"]["Operator"] = modem.getOperator();
      sendJson["GSM"]["Provider"] = modem.getProvider();
      sendJson["GSM"]["SystemMode"] = gsmDetails[SYSTEM_MODE];
      sendJson["GSM"]["MCC"] = gsmDetails[MCC];
      sendJson["GSM"]["MNC"] = gsmDetails[MNC];

      if (String(gsmDetails[SYSTEM_MODE]).equals("GSM"))
      {
        sendJson["GSM"]["LAC"] = gsmDetails[AC];
        sendJson["GSM"]["CellID"] = gsmDetails[CELLID];
      }
      else if (String(gsmDetails[SYSTEM_MODE]).equals("LTE"))
      {
        sendJson["GSM"]["TAC"] = gsmDetails[AC];
        sendJson["GSM"]["SCellID"] = gsmDetails[CELLID];
      }
      sendJson["GSM"]["SignalQuality"] = signalQuality;

      sendJson["GPS"]["FixMode"] = info.isFix;
      sendJson["GPS"]["Latitude"] = info.latitude;
      sendJson["GPS"]["Longitude"] = info.longitude;
      sendJson["GPS"]["Altitude"] = info.altitude;
      sendJson["GPS"]["Speed"] = info.speed * 1.852;
      sendJson["GPS"]["Satellites"] = info.gps_satellite_num;
      sendJson["GPS"]["Course"] = info.course;
      sendJson["GPS"]["PDOP"] = info.PDOP;
      sendJson["GPS"]["HDOP"] = info.HDOP;
      sendJson["GPS"]["VDOP"] = info.VDOP;
      sendJson["GPS"]["Odometer"] = gpsOdometer;
      sendJson["GPS"]["LastFix"] = elapsedFixTime;

      sendJson["IMU"]["IMUYaw"] = ypr[0] * 180 / M_PI;
      sendJson["IMU"]["IMUPitch"] = ypr[1] * 180 / M_PI;
      sendJson["IMU"]["IMURoll"] = ypr[2] * 180 / M_PI;
      sendJson["IMU"]["IMUAccelX"] = aaWorld.x / 16384.00;
      sendJson["IMU"]["IMUAccelY"] = aaWorld.y / 16384.00;
      sendJson["IMU"]["IMUAccellZ"] = aaWorld.z / 16384.00;

      sendJson["Storage"]["UsedSpace"] = (float)(SD.usedBytes() / (1024 * 1024));

      sendJson["Vehicle"]["IgnitionStatus"] = getIgnition() ? "Active" : "Inactive";
      sendJson["Vehicle"]["IsCrashDetected"] = crashDetection ? "Yes" : "No";
      sendJson["Vehicle"]["IsImmobilized"] = getMotorMovement() ? "Active" : "Inactive";
      sendJson["Vehicle"]["IsTowed"] = info.speed > 0 && rpm == 0 ? "Yes" : "No";
      sendJson["Vehicle"]["TheftDetected"] = rpm > 0 && !getIgnition() ? "Yes" : "No";
      sendJson["Vehicle"]["RPM"] = rpm;
      sendJson["Vehicle"]["Odometer"] = odometer;
      sendJson["Vehicle"]["VehicleNumber"] = vehicleNumber;

      sendJson["IOT"]["VendorID"] = vendorID;
      sendJson["IOT"]["HardwareNumber"] = hardwareNumber;
      sendJson["IOT"]["FirmwareVersion"] = firmwareVersion;
      sendJson["IOT"]["SerialNumber"] = serialNumber;
      sendJson["IOT"]["Temperature"] = mpu.getTemperature() / 340.00 + 36.53;

      sendJson["Driver"]["HarshAcceleration"] = harshAcceleration;
      sendJson["Driver"]["HarshDeceleration"] = harshDeceleration;
      sendJson["Driver"]["RashTurning"] = rashTurning;

      sendJsonData(sendJson);
    }

    if ((millis() - millisLog) > logInterval)
    {
      millisLog = millis();
      writeLogs(rtc, info, ypr, aaWorld, harshAcceleration, harshDeceleration, rashTurning, crashDetection);
    }

    harshAcceleration = false;
    harshDeceleration = false;
    rashTurning = false;
    crashDetection = false;
  }
}
