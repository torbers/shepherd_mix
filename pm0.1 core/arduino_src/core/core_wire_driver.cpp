
#include "core_wire_driver.h"

CoreWireDriver::CoreWireDriver(TwoWire& wire_arg) {
  ModuleCount = 0;
  Wire = &wire_arg;
}

bool CoreWireDriver::begin(uint8_t wire_sda, uint8_t wire_scl, uint32_t wire_clock) {
  Wire->setClock(wire_clock);
  return Wire->begin(wire_sda, wire_scl);
}

uint8_t CoreWireDriver::getName(uint8_t address) {
  Wire->beginTransmission(address);
  Wire->write(0xFF);
  Wire->write(0x01);
  Wire->endTransmission();

  Wire->requestFrom(address, 1);
  if (Wire->available()) {return Wire->read();}
  return 0x00;
}

uint8_t CoreWireDriver::findNextNewModule() {

  Serial.print("Beginning on 0d");
  Serial.println(BeginningAddress);

  Wire->beginTransmission(BeginningAddress);
  Wire->write(0xFF);
  Wire->write(0x01);
  Wire->write(NextNewAddress);
  Wire->endTransmission();

  delay(5);

  Serial.print("Testing 0d");
  Serial.println(NextNewAddress);

  Wire->beginTransmission(NextNewAddress);
  Wire->write(0xFF);
  Wire->write(0x00);
  Wire->write(0x00);
  Wire->write(NextNewAddress);
  Wire->endTransmission();

  Wire->beginTransmission(NextNewAddress);
  Wire->write(0xFF);
  Wire->endTransmission();

  delay(5);

  uint8_t recieved_addr = 0x00;
  uint8_t recieved_type;

  Wire->requestFrom(NextNewAddress, 3);

  if (Wire->available()) {
    Serial.println("W avail...");
    uint8_t wr = Wire->read();
    Serial.println(wr);

    if (wr == 0x00) {
      Serial.println("Recieved 0x00");
      recieved_addr = Wire->read();
      Serial.print("Recieved addr 0d");
      Serial.println(recieved_addr);

      recieved_type = Wire->read();
    }
  }

  if (recieved_addr == NextNewAddress) {
    // connection confirmed - create module

    uint8_t module_type = getName(NextNewAddress);

    switch(module_type) {
      case HALL:
        Chain[ModuleCount] = new Hall(NextNewAddress);
        Chain[ModuleCount]->PositionInChain = ModuleCount;
        break;

      case MODS:
        Chain[ModuleCount] = new Mods(NextNewAddress);
        Chain[ModuleCount]->PositionInChain = ModuleCount;
        break;

      default:
        return 0x00;
    }

    ModuleCount++;

    NextNewAddress++;
    return NextNewAddress - 1;
  }
  else {return 0x00;}
}

uint8_t updateModules() {
  for (uint8_t m = 0; Chain[m]->codename != 0xFF; m++;) {

  }
  return 0x01;
}
