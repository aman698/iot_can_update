
#include "vehiclecontrol.h"

static const byte relayIgnition = 37;
static const byte relayMotor = 36;

void initVehicleControlPins()
{
    pinMode(relayIgnition, OUTPUT);
    pinMode(relayMotor, OUTPUT);
}

void setIgnitionLock(bool active)
{
    digitalWrite(relayIgnition, active);
}

void setMotorMovementLock(bool active)
{
    digitalWrite(relayMotor, !active);
}

bool getIgnition()
{
    return digitalRead(relayIgnition) == 0;
}

bool getMotorMovement()
{
    return digitalRead(relayMotor) == 0;
}