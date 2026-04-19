
void README() {
  return;

  /*
  KH-24 I2C Hall-effect (magnetic) keyboard peripheral module
  Part of Shepherd System
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
        0x08 WRITE 0xFF 0x01 0x09 STOP
      
      Controller tests address
        0x09
        WRITE
        0xFF (switch to control register)
        0x00 (pointer = 0x00)
        0x00 (write 0x00)
        0x09 (write expected address)
      
      Controller sets pointer
        0x09 WRITE 0xFF (switch to control register)
      
      Controller requests 2 bytes
        0x00 0x09
      
      Recieves 0x09 as expected
    
    Device startup
      Begin Wire on 0x08
      Recieve message from controller
      Switch to assigned address
      Enable power on next device


  Control register:
  0x00 0x00
  0x01 I2C Address
  0x02 0x01 (Module codename)

  // TODO:
  0x03 Enable power = 0x01
  0x04 Enable high calibration (all keys) = 0x01
  0x05 Enable low calibration = 0x01
  0x06 Key to calibrate
  0x08..0xA LED color

  */
}

#include <Wire.h>
#include <EEPROM.h>

//#define DEBUG_SERIAL
//#define WIRE_DEBUG

#define SELA 11
#define SELB 12
#define SELC 13

#define RA 0
#define RB 1
#define RC 2
#define RD 3

#define DEVICE_ADDR 0x69
#define REGISTER_SIZE 0xFF
#define C_REGISTER_SIZE 0x0F

#define THRESH_HIGH 240
#define THRESH_LOW  50

int hallValue;
unsigned long us;
unsigned long nus;

uint16_t allValuesNowA[7];
uint16_t allValuesNowB[7];
uint16_t allValuesNowC[7];
uint16_t allValuesNowD[7];

volatile uint8_t DeviceRegisters[REGISTER_SIZE];
volatile uint8_t ControlRegisters[C_REGISTER_SIZE];
volatile uint8_t WirePointer = 0;

volatile uint8_t NewAddress = 0x00;
volatile uint8_t af = 0x00;

int PowerSwitchPin = 16;
int CalibrationEnablePin = 15;

// MIDI message types
#define KEY_ON       0x80;
#define KEY_OFF      0x90;
#define AFTERTOUCH   0xA0;

// Codes from changes register
#define FULLY_PRESSED             0b11
#define PARTIAL_AFTER_FULL        0b10
#define PARTIAL_AFTER_UNPRESSED   0b01
#define UNPRESSED                 0b00

// Assigning numbers to modules' codenames in Shepherd System
#define KT05   0x00 // This control module
#define KH24   0x01 // 24-key hall keyboard
#define KC02   0x02 // 2-channel pitch bend/mod/cc controller


// I2C Functions

uint8_t startupAddressAlloc(){
  Wire.begin(0x08);
  Serial.println("Wire started on 0x08");
  Wire.onReceive(startupWireHandler);
  Serial.println("Handler set");
  while (!af) {Serial.println(millis()); delay(10);}
  Serial.println(NewAddress);
  
  return NewAddress;
}


void startupWireHandler(int nb) {
  if (Wire.read() == 0xFF) { 
    #ifdef WIRE_DEBUG
      Serial.println("Recieved 0xFF");
    #endif
    if (Wire.read() == 0x01) {
      #ifdef WIRE_DEBUG
        Serial.println("Recieved 0x01");
      #endif
      NewAddress = Wire.read();
      #ifdef WIRE_DEBUG
        Serial.print("Recieved new addr: 0d");
        Serial.println(NewAddress);
      #endif
      delay(1);
      #ifdef WIRE_DEBUG
        Serial.println("Writing 0xd");
      #endif
      Wire.write(0x00); 
      #ifdef WIRE_DEBUG
        Serial.print("Writing 0d");
        Serial.println(NewAddress);
      #endif
      Wire.write(NewAddress);
      delay(10);
      af = 1;
    }
  }
}


void receiveHandler(int numbytes){
  // Handle writes to registers from I2C controller
  // Modified by me from original by SpenceKonde
  uint8_t ReadData;
  Wire.getBytesRead();
  WirePointer = Wire.read();

  #ifdef WIRE_DEBUG
    Serial.println();
    Serial.println("Recieve handler");

    Serial.println(millis());
    Serial.print("Pointer: 0d");
    Serial.println(WirePointer);
  #endif

  if (WirePointer == 0xFF) {
      #ifdef WIRE_DEBUG
        Serial.println("Control register selected");
      #endif

      numbytes--;
      if (Wire.available()){
        WirePointer = Wire.read();
      }
      else {return;}

      #ifdef WIRE_DEBUG
        Serial.print("Ctl addr: 0d");
        Serial.println(WirePointer);
      #endif

      numbytes--;
      while (numbytes > 0) {
        ReadData = Wire.read();
        #ifdef WIRE_DEBUG
          Serial.print("Recieved into register: 0d"); Serial.println(ReadData);
        #endif
        ControlRegisters[WirePointer] = ReadData;
      WirePointer++;
      numbytes--;
    }
    return;
  }

  numbytes--;
    while (numbytes > 0) {
      ReadData = Wire.read();
      DeviceRegisters[WirePointer] = ReadData;
      WirePointer++;
      numbytes--;
    }
}


void requestHandler(){
  uint8_t bytes_read = Wire.getBytesRead();
  #ifdef WIRE_DEBUG
    Serial.println();
    Serial.println("Request handler");
    Serial.print("Bytes read: ");
    Serial.println(bytes_read);
    Serial.print("Pointer: ");
    Serial.println(WirePointer);
  #endif

  WirePointer += bytes_read;
  // Update last-read values
  if (WirePointer == 0xC0) {
    for (uint8_t k = 0; k < 0x20; k++) {DeviceRegisters[k + 0xA0] = DeviceRegisters[k + 0x20];}
  }

  if (WirePointer == 0xFF) {
    #ifdef WIRE_DEBUG
      Serial.println("Control register selected for read");
    #endif
    for (byte i = 0; i < C_REGISTER_SIZE; i++) {
      if (i > C_REGISTER_SIZE) {return;}
      #ifdef WIRE_DEBUG
        Serial.print("Writing to Wire: 0d");
        Serial.println(ControlRegisters[i]);
      #endif
      Wire.write(ControlRegisters[i]);
    }
    return;
  }



  for (byte i = 0; i < REGISTER_SIZE; i++) {
    if (WirePointer + i > REGISTER_SIZE) {return;}
    Wire.write(DeviceRegisters[WirePointer + i]);
    #ifdef WIRE_DEBUG
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
  digitalWrite(SELA, channel & 0x01);
  digitalWrite(SELB, (channel >> 1) & 0x01);
  digitalWrite(SELC, (channel >> 2) & 0x01);
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
  //#ifdef DEBUG_SERIAL
    Serial.print("Calibrating key: ");
    Serial.println((int) value_addr);
  //#endif

  // halt until key pressed
  while (constrain(map(DeviceRegisters[value_addr], 0x00, DeviceRegisters[value_addr + 0x40], 0x00, 0xFF), 0x00, 0xFF) > 200) {
    getAnalogValues();
  }
  
  //#ifdef DEBUG_SERIAL
    Serial.print("Key pressed...");
  //#endif
  
  uint8_t value = 0xFF;
  for (int s = 0; s < 500; s++) {
    getAnalogValues();
    if (DeviceRegisters[value_addr] < value) {value = DeviceRegisters[value_addr];}
    delay(1);
  }

  //#ifdef DEBUG_SERIAL
    Serial.print("Lowest level: ");
    Serial.println((int) value);
  //#endif

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

  calibrateKeyLow(0x00);
  calibrateKeyLow(0x01);
  calibrateKeyLow(0x02);
  calibrateKeyLow(0x03);

}

// THRESH_HIGH and THRESH_LOW determine endpoints
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
    if (pointer >= REGISTER_SIZE - 1) {return;}
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

  pinMode(CalibrationEnablePin, INPUT);

  // Serial debug setup
  //#ifdef DEBUG_SERIAL
  Serial.begin(115200);
  Serial.println("Serial ok");
  //#endif
  
  // Do address allocation; begin Wire
  delay(100);
  ControlRegisters[0x02] = KH24; // Set module type
  while (!NewAddress) {startupAddressAlloc();}
  Wire.end();
  Serial.print("Beginning wire on 0x");
  Serial.println(NewAddress);
  Wire.begin(NewAddress);

  // Handlers for I2C
  Serial.println("Assigning handlers");
  Wire.onReceive(receiveHandler);
  Wire.onRequest(requestHandler);

  // Calibration
  for(uint8_t i = 0x40; i < 0x60; i++) {DeviceRegisters[i] = 0xFF;}
  for(uint8_t i = 0x60; i < 0x80; i++) {DeviceRegisters[i] = 0x00;}

  getAnalogValues();

  if (EEPROM.read(0x20) == 255 or digitalRead(CalibrationEnablePin)) {
    doCalibrationRoutine();
    writeStoredCalibration();
  }
  else {getStoredCalibration();}

  Serial.println("Beginning main loop");

}


void loop() {
  us = nus;
  nus = micros();
  digitalWrite(PowerSwitchPin, ControlRegisters[0x02]);
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

  //Serial.print("us:");
  //Serial.println(nus - us);
}
