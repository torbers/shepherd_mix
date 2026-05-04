
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
        updateHall(WireDriver->Chain[m]);
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

uint8_t ModuleDriver::updateHallDebug(Module* hall) {
  //delay(500);
  screen->tft.fillScreen(0x07E0);
  screen->tft.setCursor(0, 0);


  static uint8_t all_reg[0xFF];


  for(uint8_t p = 0; p<8; p++){
    screen->tft.print(p);

    WireDriver->Wire->beginTransmission(hall->address);
    WireDriver->Wire->write(0x20 * p);
    WireDriver->Wire->endTransmission();

    WireDriver->Wire->requestFrom(hall->address, 0x20);

    uint8_t kn = 0;
    while (WireDriver->Wire->available()) {
      all_reg[0x20 * p + kn] = WireDriver->Wire->read();
      kn++;
    }

  Serial.println();
  }



  static uint8_t changes_register[CHANGES_LENGTH];

  for (int i = 0; i<CHANGES_LENGTH; i++){changes_register[i] = all_reg[i+0xC0];}

  uint8_t ch = hall->PositionInChain;
  
  
  unsigned long dt = hall->MicrosSinceLastRead;
  Serial.println(dt);

  screen->tft.println("values:");

  screen->tft.println("a values:");
  for (uint8_t v = 0; v < 0x04; v++) {screen->tft.print(all_reg[v], HEX);}
  screen->tft.println();
  screen->tft.println("calib:");
  for (uint8_t v = 0x20; v < 0x24; v++) {screen->tft.print(all_reg[v], HEX);}
  screen->tft.println();
  screen->tft.println("hi:");
  for (uint8_t v = 0x40; v < 0x44; v++) {screen->tft.print(all_reg[v], HEX);}
  screen->tft.println();
  screen->tft.println("low:");
  for (uint8_t v = 0x60; v < 0x64; v++) {screen->tft.print(all_reg[v], HEX);}
  screen->tft.println();
  screen->tft.println("c:");
  for (uint8_t v = 0xC0; v < 0xC8; v++) {screen->tft.print(all_reg[v], HEX);}
  screen->tft.println();


  for (uint8_t i = 0; i < (CHANGES_LENGTH >> 2); i++){
    
    if (changes_register[i*4] == 0x00) {screen->tft.println("if"); break;}
    screen->tft.println("cr");

    uint8_t key_number = changes_register[i*4] - 0x80;

    uint8_t old_state = changes_register[i*4+1] >> 4;

    uint8_t new_state = changes_register[i*4+1] - (old_state << 4);

    uint8_t new_position = changes_register[i*4+2] - 0x80;

    uint8_t old_position = changes_register[i*4+3] - 0x80;

    uint8_t true_key = sortKeys(key_number);
    
    if (true_key == 0xFF) {return 0xFF;}

    if (new_state == 0b11) {midi->sendMidiDataUSB(0x90, ch, true_key, 0x7F);}
    if (new_state == 0b00) {midi->sendMidiDataUSB(0x80, ch, true_key, 0x00);}
    
    
  }

  return 0x00;
}


uint8_t ModuleDriver::updateHall(Module* hall) {
  //delay(500);
  //screen->tft.fillScreen(0x07E0);
  //screen->tft.setCursor(0, 0);


  static uint8_t changes_register[CHANGES_LENGTH];



  WireDriver->Wire->beginTransmission(hall->address);
  WireDriver->Wire->write(0xC0);
  WireDriver->Wire->endTransmission();

  WireDriver->Wire->requestFrom(hall->address, 0x20);

  uint8_t kn = 0;
  while (WireDriver->Wire->available()) {
    changes_register[kn] = WireDriver->Wire->read();
    kn++;
  }


  uint8_t ch = hall->PositionInChain;
  
  
  unsigned long dt = hall->MicrosSinceLastRead;
  //Serial.println(dt);


  for (uint8_t i = 0; i < (CHANGES_LENGTH >> 2); i++){
    
    if (changes_register[i*4] == 0x00) {break;}//screen->tft.println("if"); break;}
    //screen->tft.println("cr");

    uint8_t key_number = changes_register[i*4] - 0x80;

    uint8_t old_state = changes_register[i*4+1] >> 4;

    uint8_t new_state = changes_register[i*4+1] - (old_state << 4);

    uint8_t new_position = changes_register[i*4+2] - 0x80;

    uint8_t old_position = changes_register[i*4+3] - 0x80;

    uint8_t true_key = sortKeys(key_number);
    
    if (true_key == 0xFF) {return 0xFF;}
    //uint8_t tv = 127 - constrain(new_position - 127, 0, 127);


    //if (old_state == 0b00 and (new_state == 0b01 or new_state == 0b10 or new_state == 0b11) ) {midi->sendMidiDataUSB(0x90, ch, true_key, 127);}
    if (new_state = 0b11) {midi->sendMidiDataUSB(0x90, ch, true_key, 127);}
    if (new_state == 0b00) {midi->sendMidiDataUSB(0x80, ch, true_key, 0x00);}
    
    
  }

  return 0x00;
}


/*
uint8_t* ModuleDriver::getChangesHall(uint8_t addr) {
  WireDriver->Wire->beginTransmission(addr);
  WireDriver->Wire->write(0xC0);
  WireDriver->Wire->endTransmission();

  WireDriver->Wire->requestFrom(addr, CHANGES_LENGTH);

  uint8_t *return_reg = (uint8_t*) malloc(CHANGES_LENGTH);
  uint8_t i = 0;
  while (WireDriver->Wire->available()) {
    return_reg[i] = WireDriver->Wire->read();
    Serial.print(return_reg[i], BIN); Serial.print(" ");
    i++;
  }


  Serial.println();

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
    if (true){//!((i == 0) || (i == 4) || (i == 7) || (i == 7) || (i == 15) || (i == 16) || (i == 20) || (i == 23) || (i==31)) ) {
      return_reg[kn] = WireDriver->Wire->read();
      Serial.print(return_reg[kn], HEX); Serial.print(" ");
      kn++;
    }
    else {WireDriver->Wire->read();}

    i++;
  }

  Serial.println();

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
  Serial.println("Printing calib values");
  while (WireDriver->Wire->available()) {
    if (true){//!((i == 0) || (i == 4) || (i == 7) || (i == 7) || (i == 15) || (i == 16) || (i == 20) || (i == 23) || (i==31)) ) {
      return_reg[kn] = WireDriver->Wire->read();
      Serial.print(return_reg[kn], HEX); Serial.print(" ");
      kn++;
    }
    else {WireDriver->Wire->read();}

    i++;
  }

  Serial.println();

  return return_reg;
}

uint8_t* ModuleDriver::getAllValuesHall(uint8_t addr) {
  uint8_t *return_reg = (uint8_t*) malloc(0xFF);

  uint8_t kn = 0;
  for(uint8_t p = 0; p<8; p++){

    screen->tft.print(p);

    WireDriver->Wire->beginTransmission(addr);
    WireDriver->Wire->write(0x20 * p);
    WireDriver->Wire->endTransmission();

    WireDriver->Wire->requestFrom(addr, 0x20);

    while (WireDriver->Wire->available()) {
      if (true){//!((i == 0) || (i == 4) || (i == 7) || (i == 7) || (i == 15) || (i == 16) || (i == 20) || (i == 23) || (i==31)) ) {
        return_reg[kn + 0x20 * p] = WireDriver->Wire->read();
        Serial.print(return_reg[kn + 0x20 * p], HEX); Serial.print(" ");
        kn++;
      }
      else {WireDriver->Wire->read();}
    }

  Serial.println();
  }

  screen->tft.print("values got");
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


    if (new_state == 0b11) {midi->sendMidiDataUSB(0x90, 0x00, true_key, 0x7F);}
    if (new_state == 0b00) {midi->sendMidiDataUSB(0x80, 0x00, true_key, 0x00);}
    
    
  }
  //Serial.println("Changes complete 0x00");
  free(changes_register);
  return 0x00;

}
*/


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


  static uint8_t lo_register[0x20];


  Serial.println("Bottom out calib");

  WireDriver->Wire->beginTransmission(addr);
    WireDriver->Wire->write(0x00);
    WireDriver->Wire->endTransmission();

    WireDriver->Wire->requestFrom(addr, 0x20);

    uint8_t kn = 0;
    while (WireDriver->Wire->available()) {
      lo_register[kn] = WireDriver->Wire->read();
      kn++;
    }
  
  WireDriver->Wire->beginTransmission(addr);
  WireDriver->Wire->write(0x60);
  for(int k = 0; k <= kn; k++) {
    WireDriver->Wire->write(lo_register[k]);
  }

  return 0x00;
  for (uint8_t true_key = 0; true_key < 24; true_key++) {
    uint8_t key_number = rvs_key_map[true_key];//*std::find(key_map, key_map + 0x20, true_key);

    Serial.print("Key tru "); Serial.println(true_key);
    Serial.print("Key no  "); Serial.println(key_number);

    screen->tft.print("Press k "); screen->tft.println(true_key); screen->tft.println(key_number);
    Serial.println(WireDriver->setControlRegVariable(addr, 0x06, key_number));
    WireDriver->setControlRegVariable(addr, 0x05, 0x01);
    delay(2000);

    screen->tft.println("OK");

    WireDriver->Wire->beginTransmission(addr);
    WireDriver->Wire->write(0x60);
    WireDriver->Wire->endTransmission();

    WireDriver->Wire->requestFrom(addr, 0x20);

    uint8_t kn = 0;
    while (WireDriver->Wire->available()) {
      lo_register[kn] = WireDriver->Wire->read();
      kn++;
    }


    screen->tft.print("New val: ");
    screen->tft.println(lo_register[key_number]);
    
    WireDriver->Wire->beginTransmission(addr);
    WireDriver->Wire->write(0x00);
    WireDriver->Wire->endTransmission();

    WireDriver->Wire->requestFrom(addr, 0x20);

    kn = 0;
    while (WireDriver->Wire->available()) {
      lo_register[kn] = WireDriver->Wire->read();
      kn++;
    }
    
    screen->tft.print("a val: ");
    screen->tft.println(lo_register[key_number]);
    
    delay(2000);

    Serial.println();

    screen->tft.fillScreen(0x07E0);
    screen->tft.setCursor(0, 0);

  }

  delay(1000);
  return 0x00;
}