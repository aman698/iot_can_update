# IOT_module_firmware

## Table of Contents
1. [Overview](#overview)
2. [Authentication](#authentication)
3. [Telemetry Data](#telemetry-data)
4. [Logging](#logging)
5. [BLE](#ble)

## Overview
This is an advanced telemetry module, which is precise, reliable and secure. It provides data to the server through MQTTS protocol, and utilizes a powerful cryptographic co-processor for authentication, ensuring security and prevention of unauthorized access.

## Authentication
To initiate device authentication, the server will publish this JSON data to the device:
```json
{
    "isAuthActive": true/false
}
```
The device will then generate a signature based off the serial number of the device, and it will publish a JSON data to the server like so:
```json
{
    "serialNumber": "...",
    "signature": "..."
}
```

After a successful authentication, the server will generate a 32 byte challenge, and publish this JSON data to the device:
```json
{
    "isFirstAuthSuccess": true/false,
    "challenge": "..."
}
```

The device will then generate a signature based off the challenge, and it will publish a JSON data to the server like so:
```json
{
    "signature": "..."
}
```

Depending upon the signature verification, the server will publish this JSON data to the device:
```json
{
    "isSecondAuthSuccess": true/false
}
```
After which, the device will start publishing telemetry data.

## Telemetry data
The device will publish telemetry data at 10 seconds interval (when the device is active) in the following JSON structure:
```json
{
    TODO
}
```

This structure contains nested data, based on the type of data gathered from different components/sensors:
### Battery
- It provides voltage values of both internal and external battery.
- It lets you know if the external battery is connected or not.

### GSM
The data gathered from GSM modem are:
- ICCID number
- IMEI number
- IMSI number
- Network operator
- Network provider
- Network type
- MCC
- MNC
- LAC/TAC
- CellID
- Signal quality

### GPS
The data gathered from modem are:
- Fix mode
- Longitude
- Latitude
- Altitude
- GPS Speed
- No of satellites
- Course
- PDOP
- HDOPmodem
- VDOP
- GPS odometer
- Elapsed time since last fix (in seconds)

### IMU
The data gathered from IMU sensors are:
- Yaw
- Roll
- Pitch
- Acceleration X-Axis
- Acceleration Y-Axis
- Acceleration Z-Axis

### NFC
It provides the UID of the most recent NFC card used to power the vehicle.

### Storage
It provides the amount of space used for logging data.

### Vehicle
The data provided based on current vehicle status are:
- Ignition status
- Motor status
- Crash detection
- Tow detection
- RPM
- Immobilized status
- RPM odometer

## Logging
The device also has capabilities to log data and store it locally on a SD card. The data stored are:
- Latitude
- Longitude
- Altitude
- GPS Speed
- Yaw
- Pitch
- Roll
- Acceleration X-Axis
- Acceleration Y-Axis
- Acceleration Z-Axis
- Recent NFC UID

## BLE
Device can be controlled through BLE. You can send following commands:
- `deviceinfo`: Provides information about the device.
- `ignitionon`: Turns on the ignition.
- `ignitionoff`: Turns off the ignition.
- `immobilizeon`: Turns on motor immobilization.
- `immobilizeoff`: Turns off motor immobilization.
