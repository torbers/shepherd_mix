
#include "module_driver.h"

ModuleDriver::ModuleDriver(CoreWireDriver& wire_driver_arg, MidiMgmt& midi_mgmt, CoreScreen& screen_arg) {
  WireDriver = &wire_driver_arg;
  midi = &midi_mgmt;
  screen = &screen_arg;
}

//#define UPDATE_DEBUG

uint8_t ModuleDriver::updateModules() {
  #ifdef UPDATE_DEBUG
  Serial.println("Updating modules...");
  Serial.println(WireDriver->Chain[1]->codename);
  Serial.println("ok...");
  #endif
  
  
  for (uint8_t m = 0; (WireDriver->Chain[m]->codename != 0xFF) || m>MAX_MODULE_COUNT; m++) {

    #ifdef UPDATE_DEBUG
    Serial.println(m);
    #endif

    WireDriver->Chain[m]->MicrosSinceLastRead = micros() - WireDriver->Chain[m]->MicrosAtLastRead;
    WireDriver->Chain[m]->MicrosAtLastRead = micros();
    uint8_t addr = WireDriver->Chain[m]->address;

    if (WireDriver->Chain[m]->newm) {
      Serial.println("New module...");
      WireDriver->Chain[m]->newm = false;
      Serial.println("New module...");
      switch (WireDriver->Chain[m]->codename) {
        case HALL:
          Serial.println("routine...");
          doBottomOutCalibrationRoutine(addr);
          break;
        case MODS:
          break;
        default:
          return 0x00;
      }
    }

    switch (WireDriver->Chain[m]->codename) {
      case HALL:
        handleChangesRegisterHall(getChangesHall(addr));
        //getCalibValuesHall(addr);
        break;
      case MODS:
        break;
      default:
        return 0x00;
    }
  }
  return 0x01;
}


// functions for HALL

uint8_t* ModuleDriver::getChangesHall(uint8_t addr) {
  WireDriver->Wire->beginTransmission(addr);
  WireDriver->Wire->write(0xC0);
  WireDriver->Wire->endTransmission();

  WireDriver->Wire->requestFrom(addr, CHANGES_LENGTH);

  uint8_t *return_reg = (uint8_t*) malloc(CHANGES_LENGTH);
  uint8_t i = 0;
  while (WireDriver->Wire->available()) {
    return_reg[i] = WireDriver->Wire->read();
    //Serial.print(return_reg[i], HEX); Serial.print(" ");
    i++;
  }


  //Serial.println();

  return return_reg;
}

uint8_t* ModuleDriver::getValuesHall(uint8_t addr) {
  WireDriver->Wire->beginTransmission(addr);
  WireDriver->Wire->write(0x00);
  WireDriver->Wire->endTransmission();

  WireDriver->Wire->requestFrom(addr, 0x20);

  uint8_t return_reg[0x20];
  uint8_t i = 0;
  uint8_t kn = 0;
  while (WireDriver->Wire->available()) {
    if (!(i == 0 | i == 4 | i == 7 | i == 15 | i == 16 | i == 20 | i == 23)) {
      return_reg[kn] = WireDriver->Wire->read();
      kn++;
    }
    else {WireDriver->Wire->read();}
    //Serial.print(return_reg[kn], HEX); Serial.print(" ");
    i++;
  }

  //Serial.println();

  return return_reg;
}

uint8_t* ModuleDriver::getCalibValuesHall(uint8_t addr) {
  WireDriver->Wire->beginTransmission(addr);
  WireDriver->Wire->write(0x20);
  WireDriver->Wire->endTransmission();

  WireDriver->Wire->requestFrom(addr, 0x20);

  uint8_t return_reg[0x20];
  uint8_t i = 0;
  uint8_t kn = 0;
  //Serial.println("Printing calib values");
  while (WireDriver->Wire->available()) {
    if (true){//!((i == 0) || (i == 4) || (i == 7) || (i == 7) || (i == 15) || (i == 16) || (i == 20) || (i == 23) || (i==31)) ) {
      return_reg[kn] = WireDriver->Wire->read();
      //Serial.print(return_reg[kn], HEX); Serial.print(" ");
      kn++;
    }
    else {WireDriver->Wire->read();}

    i++;
  }

  //Serial.println();

  return return_reg;
}

uint8_t ModuleDriver::handleChangesRegisterHall(uint8_t *changes_register){
  //unsigned long dt = WireDriver->Chain[chain_pos]->MicrosSinceLastRead;
  //Serial.println(dt);

  for (uint8_t i = 0; i < (CHANGES_LENGTH >> 2); i++){
    if (changes_register[i*4] == 0x00) {break;}

    uint8_t key_number = changes_register[i*4] - 0x80;

    uint8_t old_state = changes_register[i*4+1] >> 4;

    uint8_t new_state = changes_register[i*4+1] - (old_state << 4);

    uint8_t new_position = changes_register[i*4+2] - 0x80;

    uint8_t old_position = changes_register[i*4+3] - 0x80;

    uint8_t true_key = sortKeys(key_number);
    
    if (true_key == 0xFF) {return 0xFF;}


    if (new_state == 0x11) {midi->sendMidiDataUSB(0x90, 0x00, true_key, 0x7F);}
    if (new_state == 0x00) {midi->sendMidiDataUSB(0x80, 0x00, true_key, 0x00);}
    
    
  }
  //Serial.println("Changes complete 0x00");
  free(changes_register);
  return 0x00;

}

uint8_t* ModuleDriver::keyToNoteOctHall(uint8_t key){
  uint8_t output[2];
  output[0] = key;
  output[1] = 0;
  return output;
}

uint8_t ModuleDriver::sortKeys(uint8_t key_number) {
  uint8_t i = key_number;

  //if (!((i == 0) || (i == 4) || (i == 7) || (i == 7) || (i == 15) || (i == 16) || (i == 20) || (i == 23) || (i==31))) {return 0xFF;}


  uint8_t true_key = key_map[i];

  return true_key;
}

uint8_t ModuleDriver::doBottomOutCalibrationRoutine(uint8_t addr){

  Serial.print("Addr bo: "); Serial.println(addr, HEX);

  WireDriver->Wire->beginTransmission(addr);
  WireDriver->Wire->write(0x00);
  WireDriver->Wire->requestFrom(addr, 0xFF);

  while(WireDriver->Wire->available()) {
    Serial.print(WireDriver->Wire->read(), HEX); Serial.print(" ");
  }


  Serial.println("Bottom out calib");
  for (uint8_t true_key = 0; true_key < 24; true_key++) {
    uint8_t key_number = rvs_key_map[true_key];//*std::find(key_map, key_map + 0x20, true_key);

    Serial.print("Key tru "); Serial.println(true_key);
    Serial.print("Key no  "); Serial.println(key_number);

    screen->tft.print("Press k "); screen->tft.println(true_key); screen->tft.println(key_number);
    Serial.println(WireDriver->setControlRegVariable(addr, 0x06, key_number));
    WireDriver->setControlRegVariable(addr, 0x05, 0x01);
    delay(2000);

    screen->tft.println("OK");

    delay(500);

    Serial.println();

    screen->tft.fillScreen(0x07E0);
    screen->tft.setCursor(0, 0);

  }

  WireDriver->Wire->beginTransmission(addr);
  WireDriver->Wire->write(0x00);
  WireDriver->Wire->requestFrom(addr, 0xFF);
  Wire.endTransmission();

  while(WireDriver->Wire->available()) {
    Serial.print(WireDriver->Wire->read(), HEX); Serial.print(" ");
  }

  Serial.println();



  delay(1000);
  return 0x00;
}