/*
I2C Hall-effect (magnetic) keyboard peripheral
by Sam Kothe (torbers)

Makes available states, positions, and state-changes of 32 keys over I2C.


Hardware:
  Microcontroller: ATTINYx16
  Analog Multiplexer: CD4051
  Sensor: SLSS49E
  Output pin tied to GND via 10k resistor.


I2C Register Map:
  0x00..0x07: Raw 8-bit analog values of ROW A
  0x08..0x0F: ROW B
  0x10..0x17: ROW C
  0x18..0x1F: ROW D

  0x20..0x27: Calibrated values; max = 0xFF, min = 0x00. ROW A
  0x28..0x2F: ROW B
  0x30..0x37: ROW C
  0x38..0x3F: ROW D

  0x40..0x47: Calibration HIGH
  0x48..0x4F: ROW B
  0x50..0x57: ROW C
  0x58..0x5F: ROW D

  0x60..0x67: Calibration LOW
  0x68..0x6F: ROW B
  0x70..0x77: ROW C
  0x78..0x7F: ROW D


  11: FULLY PRESSED
  10: PARTIALLY PRESSED AFTER FULLY PRESSED
  01: PARTIALLY PRESSED AFTER UNPRESSED
  00: UNPRESSED

  0x80..0x87: Keys according to above chart (rows A, B, C, and D)
  0x88..0x8F: /
  0x90..0x97: /
  0x97..0x9F: /

  0xA0: Positions of keys when last polled by state change request (0xC0) ROW A
  0xA8: ROW B
  0xB0: ROW C
  0xB8: ROW D

  0xC0..0xC7......0xDF: State change keys

  1[0x7 KEY NUMBER]
  00[0x2 OLD STATE]00[0x2 NEW STATE]
  [0x8 CALIBRATED POSITION]
  [0x8 PREVIOUS POSITION]
  Repeat for each new state change
  ....
  Last byte 0b00000000

  Positions copied to 0xA0..0xB8 and register wiped to 0b00000000 immediately after read


I2C Address Allocation

  Controller routine (each ~200ms)
    Controller sends test address (0x09 for example)
      0x08 WRITE 0x00 0x01 0x09 STOP
    Request acknowlegement from address
      0x09 W 0x255 STOP
    If acknowlegement recieved, send ack of ack
      0x09 W 0x255 0x01
    Delay a few ms and begin communication with device & increment test address
  
  Device startup
    Begin Wire on 0x08
    Recieve message from controller
    Switch to assigned address
    Acknowlege controller on next request
      0x00 0x09
    Enable power on next device


*/

#include <Wire.h>
#include <EEPROM.h>

//#define DEBUG_SERIAL

#define SELA 11
#define SELB 12
#define SELC 13

#define RA 0
#define RB 1
#define RC 2
#define RD 3

#define DEVICE_ADDR 0x69
#define REGISTER_SIZE 0xFF

#define THRESH_HIGH 150
#define THRESH_LOW  50

int hallValue;
unsigned long us;
unsigned long nus;

uint16_t allValuesNowA[7];
uint16_t allValuesNowB[7];
uint16_t allValuesNowC[7];
uint16_t allValuesNowD[7];

volatile uint8_t DeviceRegisters[REGISTER_SIZE];
volatile uint8_t WirePointer = 0;

volatile uint8_t NewAddress = 0x00;

int PowerSwitchPin = 5;


// I2C Functions

uint8_t startupAddressAlloc(){

  Wire.begin(0x08);
  Wire.onReceive(receiveHandler);
  Wire.onRequest(requestHandler);
  while (NewAddress == 0x00) {}
  Wire.end();
  return NewAddress;
}


void receiveHandler(int numbytes){
  // Handle writes to registers from I2C controller
  // Modified by me from original by SpenceKonde
  uint8_t ReadData;
  int nb;
  Wire.getBytesRead();
  WirePointer = Wire.read();
  numbytes--;
  nb = numbytes;
    while (numbytes > 0) {
      ReadData = Wire.read();
      if (nb == numbytes && ReadData == 0x01) {NewAddress = Wire.read(); return;}
      else {DeviceRegisters[WirePointer] = ReadData;}
      WirePointer++;
      numbytes--;
    }
}


void requestHandler(){
  uint8_t bytes_read = Wire.getBytesRead();
  #ifdef DEBUG_SERIAL
  Serial.print("Bytes read: ");
  Serial.println(bytes_read);
  Serial.print("Pointer: ");
  Serial.println(WirePointer);
  #endif

  if (WirePointer == 0xFF) {
    if (bytes_read == 0x02) {Wire.write(0x00); Wire.write(NewAddress); return;}
    
    // command to enable next module
    if (bytes_read == 0x01) {Wire.write(0x01); digitalWrite(PowerSwitchPin, LOW); return;}
  }

  WirePointer += bytes_read;
  // Update last-read values
  if (WirePointer == 0xC0) {
    for (uint8_t k = 0; k < 0x20; k++) {DeviceRegisters[k + 0xA0] = DeviceRegisters[k + 0x20];}
  }


  for (byte i = 0; i < REGISTER_SIZE; i++) {
    if (WirePointer + i > REGISTER_SIZE) {return;}
    Wire.write(DeviceRegisters[WirePointer + i]);
    #ifdef DEBUG_SERIAL
    Serial.print("Reading to I2C: ");
    Serial.println(WirePointer + i);
    Serial.print("Result: ");
    Serial.println(DeviceRegisters[WirePointer + i]);
    #endif

    // Clear queue
    if (WirePointer + i >= 0xC0) {DeviceRegisters[WirePointer + i] = 0x00;}
  }

}


// Key reading & calibration functions

void getAnalogValues() {
  for (int r = 0; r < 8; r++) {
    // Set 4051 mux channel
    selectMuxChannel(r);

    // Write raw values 0x00..0x1F
    // Values from ADC are 10-bit but we can clip them to 8 bit because the sensors
    // only output from 1/4 to 1/2 of 3.3V (between 256 and 511 out of 1023)
    DeviceRegisters[r + 0x00 + 0x00] = constrain(analogRead(RA) - 256, 0, 255);
    DeviceRegisters[r + 0x00 + 0x08] = constrain(analogRead(RB) - 256, 0, 255);
    DeviceRegisters[r + 0x00 + 0x10] = constrain(analogRead(RC) - 256, 0, 255);
    DeviceRegisters[r + 0x00 + 0x18] = constrain(analogRead(RD) - 256, 0, 255);

    // calculate calibrated values for all keys and store in 0x20..0x3F
    DeviceRegisters[r + 0x20 + 0x00] = keyCalibration(r + 0x00 + 0x00, r + 0x40 + 0x00, r + 0x60 + 0x00);
    DeviceRegisters[r + 0x20 + 0x08] = keyCalibration(r + 0x00 + 0x08, r + 0x40 + 0x08, r + 0x60 + 0x08);
    DeviceRegisters[r + 0x20 + 0x10] = keyCalibration(r + 0x00 + 0x10, r + 0x40 + 0x10, r + 0x60 + 0x10);
    DeviceRegisters[r + 0x20 + 0x18] = keyCalibration(r + 0x00 + 0x18, r + 0x40 + 0x18, r + 0x60 + 0x18);
  }
}


void selectMuxChannel(int channel) {
  //Serial.print(channel);
  //Serial.print(": ");

  //digitalWrite(SELA, LOW);
  //digitalWrite(SELB, LOW);
  //digitalWrite(SELC, LOW);

  digitalWrite(SELA, channel & 0x01);
  //Serial.print(channel & 0x01);
  digitalWrite(SELB, (channel >> 1) & 0x01);
  //Serial.print((channel >> 1) & 0x01);
  digitalWrite(SELC, (channel >> 2) & 0x01);
  //Serial.print((channel >> 2) & 0x01);
}


uint8_t keyCalibration(uint8_t value_addr, uint8_t high_addr, uint8_t low_addr) {
  return constrain(map(DeviceRegisters[value_addr], DeviceRegisters[low_addr], DeviceRegisters[high_addr], 0x00, 0xFF), 0x00, 0xFF);
}


void calibrateAllKeysHigh() {
  for (int r = 0; r < 0x20; r++) {
    DeviceRegisters[0x40 + r] = DeviceRegisters[0x00 + r];
  }
}


void calibrateKeyLow(uint8_t value_addr) {
  #ifdef DEBUG_SERIAL
  Serial.print("Calibrating key: ");
  Serial.println((int) value_addr);
  #endif

  // halt until key pressed
  while (constrain(map(DeviceRegisters[value_addr], 0x00, DeviceRegisters[value_addr + 0x40], 0x00, 0xFF), 0x00, 0xFF) > 200) {
    getAnalogValues();
  }
  
  #ifdef DEBUG_SERIAL
  Serial.print("Key pressed...");
  #endif
  
  uint8_t value = 0xFF;
  for (int s = 0; s < 500; s++) {
    getAnalogValues();
    if (DeviceRegisters[value_addr] < value) {value = DeviceRegisters[value_addr];}
    delay(1);
  }

  #ifdef DEBUG_SERIAL
  Serial.print("Lowest level: ");
  Serial.println((int) value);
  #endif

  DeviceRegisters[value_addr + 0x60] = value;
}


void getStoredCalibration() {
  for (uint8_t addr = 0x00; addr < 0x40; addr++) {
    DeviceRegisters[addr + 0x40] = EEPROM.read(addr);
  }
}


void writeStoredCalibration() {
  for (uint8_t addr = 0x00; addr < 0x40; addr++) {
    EEPROM.update(addr, DeviceRegisters[addr + 0x40]);
  }
}


void doCalibrationRoutine() {
  calibrateAllKeysHigh();

  calibrateKeyLow(0x01);
  calibrateKeyLow(0x02);
  calibrateKeyLow(0x03);

}

//  11: FULLY PRESSED
//  10: PARTIALLY PRESSED AFTER FULLY PRESSED
//  01: PARTIALLY PRESSED AFTER UNPRESSED
//  00: UNPRESSED

void getKeyChanges() {
  uint8_t prev_state_as_int;
  uint8_t state_as_int = 0x0;
  uint8_t key_bit;

  for (uint8_t key = 0; key < 0x20; key++) {
    key_bit = (0b00000011 & key);
    prev_state_as_int = (uint8_t) bitRead(DeviceRegisters[0x80 + (key >> 2)], key_bit << 1) + (bitRead(DeviceRegisters[0x80 + (key >> 2)], (key_bit << 1) + 1) << 1);

    if (DeviceRegisters[key + 0x20] < THRESH_HIGH) { // if key is pressed at all
      if (DeviceRegisters[key + 0x20] < THRESH_LOW) {state_as_int = 0b11;} // if key is fully pressed
      else { // if the key is partially pressed
        if (prev_state_as_int == 0b00 or prev_state_as_int == 0b01) {state_as_int = 0b01;}
        if (prev_state_as_int == 0b11 or prev_state_as_int == 0b10) {state_as_int = 0b10;}
      }
    }
    else {state_as_int = 0b00;} // if the key is unpressed

    // write new state
    bitWrite(DeviceRegisters[0x80 + (key >> 2)], key_bit << 1, state_as_int & 0b01);
    bitWrite(DeviceRegisters[0x80 + (key >> 2)], (key_bit << 1) + 1, (state_as_int >> 1) & 0b01);

    if ((prev_state_as_int != state_as_int) or (state_as_int == 0b01) or (state_as_int == 0b10)) {
      // add to queue
      addKeyToQueue(key, prev_state_as_int, state_as_int);
    }

  }
}


void printBits(uint8_t byte) {
  for (int i = 7; i >= 0; i--) {
    if (0b00000001 & (byte >> i)) {Serial.print("1 ");} else {Serial.print("0 ");}
  }
  Serial.println();
}


// Add state-change key to queue
// 1[0x7 KEY NUMBER]
// 00[0x2 OLD STATE]00[0x2 NEW STATE]
// [0x8 CALIBRATED POSITION]
// [0x8 PREVIOUS POSITION]
// 0xC0
void addKeyToQueue(uint8_t key, uint8_t old_state, uint8_t new_state) {
  uint8_t pointer = 0xC0;
  while (DeviceRegisters[pointer] != 0x00) {
    pointer++;
    if (pointer >= REGISTER_SIZE) {return;}
    }
  DeviceRegisters[pointer + 0] = key + 0b10000000;
  DeviceRegisters[pointer + 1] = (old_state << 4) + new_state;
  DeviceRegisters[pointer + 2] = DeviceRegisters[key + 0x20];
  DeviceRegisters[pointer + 3] = DeviceRegisters[key + 0xA0];
}



void setup() {
    // Multiplexer setup
  analogReadResolution(10);

  pinMode(SELA, OUTPUT);
  pinMode(SELB, OUTPUT);
  pinMode(SELC, OUTPUT);

  pinMode(RA, INPUT_PULLUP);
  pinMode(RB, INPUT_PULLUP);
  pinMode(RC, INPUT_PULLUP);
  pinMode(RD, INPUT_PULLUP);

  // Disable power to next module
  pinMode(PowerSwitchPin, OUTPUT);
  digitalWrite(PowerSwitchPin, HIGH);

  // Serial debug setup
  #ifdef DEBUG_SERIAL
  Serial.begin(115200);
  #endif
  
  // Do address allocation; begin Wire
  Wire.begin(startupAddressAlloc());

  // Handlers for I2C
  Wire.onReceive(receiveHandler);
  Wire.onRequest(requestHandler);

  // Calibration
  for(uint8_t i = 0x40; i < 0x60; i++) {DeviceRegisters[i] = 0xFF;}
  for(uint8_t i = 0x60; i < 0x80; i++) {DeviceRegisters[i] = 0x00;}

  getAnalogValues();

  if (EEPROM.read(0x20) == 255 or digitalRead(6)) {
    doCalibrationRoutine();
    writeStoredCalibration();
  }
  else {getStoredCalibration();}

}


void loop() {
  us = nus;
  nus = micros();
  // Get key values
  getAnalogValues();
  getKeyChanges();

  // Serial debug for multiplexer
  #ifdef DEBUG_SERIAL
  
  Serial.print("Max:");
  Serial.print(255);
  Serial.print(" ");
  Serial.print("Min:");
  Serial.print(0);
  Serial.print(" ");

  for (int p = 0; p < 5; p++) {
    Serial.print(p);
    Serial.print(":");
    Serial.print((int) DeviceRegisters[p + 0x20]);
    Serial.print(" ");
    //delay(1);
  }

  //Serial.print("Millis:");
  //Serial.print(millis());

  Serial.println();
  #endif

  Serial.print("us:");
  Serial.println(nus - us);
}
