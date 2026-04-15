#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "modules.h"

#define WIRE_CLOCK 1000000

#define MAX_MODULE_COUNT 16

class CoreWireDriver {
  public:
    // Module-to-module comm
    uint8_t BeginningAddress = 0x08;
    uint8_t NextNewAddress = BeginningAddress + 0x1;
    uint8_t I2CBus = 0;

    Module* Chain[MAX_MODULE_COUNT];
    TwoWire* Wire;

    uint8_t ModuleCount;

    bool begin(uint8_t wire_sda, uint8_t wire_scl, uint32_t wire_clock);
    uint8_t getName(uint8_t address);
    uint8_t findNextNewModule();

    uint8_t updateModules();

    CoreWireDriver(TwoWire& wire_arg);
};
