
#ifndef __VEHCILECONTROL_H
#define __VEHICLECONTROL_H

#include <Arduino.h>

void initVehicleControlPins();
bool getIgnition();
bool getMotorMovement();
void setIgnitionLock(bool active);
void setMotorMovementLock(bool active);

#endif