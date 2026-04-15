#pragma once

#include <Wire.h>
#define WIRE_CLOCK 1000000

class CoreWireDriver {
  public:
    TwoWire Wire;
    bool begin(uint8_t wire_sda, uint8_t wire_scl, uint32_t wire_clock);
};

bool CoreWireDriver::begin(uint8_t wire_sda, uint8_t wire_scl, uint32_t wire_clock = WIRE_CLOCK) {
  Wire.setClock(wire_clock);
  return Wire.begin(wire_sda, wire_scl);
}