#pragma once

#include <Arduino.h>

#include "modules.h"
#include "midi_mgmt.h"
#include "core_wire_driver.h"

class ModuleDriver {
  public:
    CoreWireDriver WireDriver;

    uint8_t updateModules();

    ModuleDriver(CoreWireDriver wire_driver_arg);
};