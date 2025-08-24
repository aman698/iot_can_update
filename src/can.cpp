#include "can.h"

// MCP2515 object
MCP2515 mcp(CAN_CS, 8000000, &SPI);
struct can_frame canMsg;

void CAN_Init() {
  // Initialize SPI
  SPI.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN_CS);

  // Initialize MCP2515
  mcp.reset();
  mcp.setBitrate(CAN_500KBPS, MCP_8MHZ);

  // Optional: filter to accept only ID 0x123
  mcp.setFilterMask(MCP2515::MASK0, false, 0x7FF);  
  mcp.setFilter(MCP2515::RXF0, false, 0x123);

  mcp.setNormalMode();

  Serial.println("MCP2515 Initialized Successfully in NORMAL mode!");
}

bool CAN_Read() {
  if (mcp.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    return true;
  }
  return false;
}

void CAN_PrintFrame() {
  Serial.print("ID: 0x");
  Serial.print(canMsg.can_id, HEX);
  Serial.print(" DLC: ");
  Serial.print(canMsg.can_dlc);
  Serial.print(" Data: ");

  for (int i = 0; i < canMsg.can_dlc; i++) {
    Serial.print(canMsg.data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}
