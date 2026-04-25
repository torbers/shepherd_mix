
#include "core_wire_driver.h"

CoreWireDriver::CoreWireDriver(TwoWire& wire_arg, MidiMgmt& midi_arg) {
  ModuleCount = 0;
  Wire = &wire_arg;
  MidiCall = &midi_arg;
  Module NullModule;
  for (uint8_t i = 0; i<MAX_MODULE_COUNT; i++) {Chain[i] = &NullModule;}
}

bool CoreWireDriver::begin(uint8_t wire_sda, uint8_t wire_scl, uint32_t wire_clock) {
  Wire->setClock(wire_clock);
  return Wire->begin(wire_sda, wire_scl);
}

uint8_t CoreWireDriver::setControlRegVariable(uint8_t address, uint8_t register_address, uint8_t new_value) {
  Wire->beginTransmission(address);
  Wire->write(0xFF);
  Wire->write(register_address);
  Wire->write(new_value);
  Wire->endTransmission();

  Wire->requestFrom(address, 1);
  if (Wire->available()) {return Wire->read();}
  return 0x00;
}

uint8_t CoreWireDriver::findNextNewModule() {
  Serial.print("Beginning on 0d");
  Serial.println(BeginningAddress);

  setControlRegVariable(BeginningAddress, 0x01, NextNewAddress);


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
      Serial.print("Type: "); Serial.println(recieved_type);
    }
  }

  Serial.print("recieved_addr: "); Serial.println(recieved_addr);
  Serial.print("NextNewAddress: "); Serial.println(NextNewAddress);

  if (recieved_addr == NextNewAddress) {
    // connection confirmed - create module
    Serial.print("Creating module at "); Serial.println(recieved_addr);

    uint8_t module_type = recieved_type;

    Serial.print("module_type: "); Serial.println(module_type);

    switch (module_type) {
      case HALL:
        Chain[ModuleCount] = new Hall(NextNewAddress);
        Chain[ModuleCount]->PositionInChain = ModuleCount;

        setControlRegVariable(Chain[ModuleCount]->address, 0x04, 0x01);
        setControlRegVariable(Chain[ModuleCount]->address, 0x08, 0x00);

        Serial.println("HALL");
        break;

      case MODS:
        Chain[ModuleCount] = new Mods(NextNewAddress);
        Chain[ModuleCount]->PositionInChain = ModuleCount;
        Serial.println("MODS");
        break;

      default:
        Serial.println("NULL");
        return 0x00;
    }

    ModuleCount++;

    NextNewAddress++;
    Serial.println(NextNewAddress - 1);
    Serial.println("NEW MODULE ADDED");
    return NextNewAddress - 1;
  }
  else {Serial.println("NEW MODULE NOT ADDED"); return 0x00;}
}


