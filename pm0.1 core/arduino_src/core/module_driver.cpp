
#include "module_driver.h"

ModuleDriver::ModuleDriver(CoreWireDriver wire_driver_arg) {
  WireDriver = wire_driver_arg;
}

uint8_t ModuleDriver::updateModules() {
  for (uint8_t m = 0; WireDriver.Chain[m]->codename != 0xFF; m++) {
    WireDriver.Chain[m]-> MicrosSinceLastRead = micros() - WireDriver.Chain[m]->MicrosAtLastRead;
    WireDriver.Chain[m]->MicrosAtLastRead = micros();

    switch (WireDriver.Chain[m]->codename) {
      case HALL:
        break;
      case MODS:
        break;
      default:
        return 0x00;
    }
  }
  return 0x01;
}