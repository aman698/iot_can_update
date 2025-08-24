#ifndef CAN_H
#define CAN_H

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

// MCP2515 SPI pins
#define CAN_CS   14
#define CAN_SCK  5
#define CAN_MOSI 4
#define CAN_MISO 6

extern MCP2515 mcp;
extern struct can_frame canMsg;

void CAN_Init();               // Initialize CAN bus
bool CAN_Read();               // Read CAN message
void CAN_PrintFrame();         // Debug print

#endif
