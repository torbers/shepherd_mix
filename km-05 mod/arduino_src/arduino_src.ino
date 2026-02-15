
//void README() {
//  return;

  /*
  Dual capacitive touch slider keyboard peripheral module
  Part of Shepherd System
  by Sam Kothe (torbers)

  Senses state and position of two 6-sensor ATTiny PTC capacitive touch sliders and makes available via I2C.


  Hardware:
    Microcontroller: ATTINYx16
    Sensors tied directly to microcontroller pins (with ESD protection).


  I2C Register Map:
    0x00: Slider 1 raw
    0x01: Slider 2 raw
    0x02: Slider 1 hysteresis
    0x03: Slider 2 hysteresis

    0x04: Slider 1 magnitude
    0x05: Slider 2 magnitude
    0x06: Slider 1 magnitude hysteresis
    0x07: Slider 2 magnitude hysteresis


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
  0x03 Enable power = 0x01

  // TODO:
  0x08..0x0A LED color 1
  0x0B..0x0D LED color 2

  */
//}

#include "Hysteresis.h"
#include <Wire.h>
#include <ptc_touch.h>

#define NODE_COUNT 6


#define DEVICE_ADDR 0x69
#define REGISTER_SIZE 0xFF
#define C_REGISTER_SIZE 0x0F

volatile uint8_t DeviceRegisters[REGISTER_SIZE];
volatile uint8_t ControlRegisters[C_REGISTER_SIZE];
volatile uint8_t WirePointer = 0;

unsigned long us;
unsigned long nus;

volatile uint8_t NewAddress = 0x00;
volatile uint8_t af = 0x00;

int PowerSwitchPin = 16;

// Assigning numbers to modules' codenames in Shepherd System
#define KT05   0x00 // This control module
#define KH24   0x01 // 24-key hall keyboard
#define KC02   0x02 // 2-channel pitch bend/mod/cc controller

//#define DEBUG_SERIAL
//#define WIRE_DEBUG


// slider logic
// adapted from https://github.com/TinyCircuits/TinyCircuits-ATtiny841-Library/tree/master
class Slider {
  public:
    //Slider();
    uint8_t begin(uint8_t *sensors, uint8_t driver);
    int16_t capTouchRead(uint8_t pin);
    int16_t getPosition(int range=0xFF);
    int16_t getMaxReading();
    int16_t getMagnitude();
    uint8_t getPositionHysteresis();
    uint8_t getMagnitudeHysteresis();
    
    uint32_t duration();
    bool update();
    bool isTouched();
    
    static const uint8_t numSensors = NODE_COUNT;
    uint8_t capTouchPins[numSensors] = {};
    uint8_t driver_pin = 0;

    int16_t capTouchCal[numSensors];
    int16_t capTouchCurrent[numSensors];
    uint8_t overCalCount[numSensors];

    int16_t currentMaxReading= 0;
    uint8_t primary = 0;
    bool isTouch = 0;
    uint8_t initialTouchPos = 0;
    uint8_t finalTouchPos = 0;
    uint8_t currentTouchedPositions = 0;
    uint8_t allTouchedPositions = 0;
    uint32_t touchTimer = 0;

    cap_sensor_t s_nodes[numSensors];

    Hysteresis <int16_t> positionHysteresis = Hysteresis <int16_t> (4);
    Hysteresis <int16_t> magnitudeHysteresis = Hysteresis <int16_t> (4);
};

uint8_t Slider::begin(uint8_t sensors[numSensors], uint8_t driver) {

  driver_pin = driver;
  
  for (uint8_t p = 0; p < numSensors; p++) {
    Serial.print("Sensor no.: ");
    Serial.println(p);
    capTouchPins[p] = sensors[p];
    Serial.print("goes to pin no.: ");
    Serial.println(capTouchPins[p]);
  }

  for (uint8_t pin = 0; pin < numSensors; pin++) {
    ptc_add_selfcap_node(&s_nodes[pin], PIN_TO_PTC(capTouchPins[pin]), PIN_TO_PTC(driver_pin));
    delay(100);
  }

  ptc_process(millis());

  for (uint8_t pin = 0; pin < numSensors; pin++) {
    capTouchCal[pin] = capTouchRead(pin);
    overCalCount[pin] = 0;
    delay(100);
  }

  for (uint8_t p = 0; p < numSensors; p++) {
    Serial.print("Sensor read: ");
    Serial.println(capTouchCal[p]);
  }
  
  return 0;
}

int16_t Slider::capTouchRead(uint8_t pin) {
  //return ptc_get_node_sensor_value(&s_nodes[capTouchPins[pin] - 1]);
  return ptc_get_node_sensor_value(&s_nodes[pin]) - 512;
}

int16_t Slider::getPosition(int range) {
  float position = primary;
  if (primary < numSensors-1)
    position += (pow((float)capTouchCurrent[primary + 1] / (float)getMaxReading(),0.45)) / 2.0;
  if (primary > 0)
    position -= (pow((float)capTouchCurrent[primary - 1] / (float)getMaxReading(),0.45)) / 2.0;
  
  position = position * (float)range / float(numSensors-1);
  return position;
}

int16_t Slider::getMaxReading() {
  return currentMaxReading;
}

int16_t Slider::getMagnitude() {
  uint8_t j;
  long magnitude = 0;
  for (j = 0; j < 6; j++) {
    magnitude += pow(capTouchCurrent[j], 2);
  }
  return sqrt(magnitude);
}

uint8_t Slider::getPositionHysteresis() {
  Serial.println(getPosition());
  int16_t ph = positionHysteresis.add(getPosition());
  Serial.println(ph);
  return ph;
}

uint8_t Slider::getMagnitudeHysteresis() {
  return magnitudeHysteresis.add(getMagnitude());
}

bool Slider::update() {
  uint8_t j;
  currentMaxReading=0;
  for (j = 0; j < numSensors; j++) {
    int val = capTouchRead(j);
    if (val < capTouchCal[j]) {
      capTouchCal[j] = val;
    } else if (val > capTouchCal[j] + 1) {
      overCalCount[j]++;
      if (overCalCount[j] > 3) {
        overCalCount[j] = 0;
        capTouchCal[j] += 1;
      } else {
        //overCalCount[j]=0;
      }
    }
    capTouchCurrent[j] = val  - capTouchCal[j];  
    if (capTouchCurrent[j] > getMaxReading()) {
      currentMaxReading = capTouchCurrent[j];
      primary = j;
    }
  }

  bool completedTouch = false;

  if (currentMaxReading > 255) {
    if (!isTouch) {
      isTouch = true;
      touchTimer = millis();
      initialTouchPos = primary;
      finalTouchPos = primary;
      currentTouchedPositions = 0;
      allTouchedPositions = 0;
    } else {
      finalTouchPos = primary;
      currentTouchedPositions = 0;
      for (j = 0; j < numSensors; j++) {
        if (capTouchCurrent[j] > 64)
          currentTouchedPositions |= (1 << j);
        allTouchedPositions |= (1 << j);
      }
    }
  } else {
    if (isTouch) {
      isTouch = false;
      touchTimer = millis() - touchTimer;
      completedTouch = true;
    }
  }

  return completedTouch;
}

bool Slider::isTouched() {
  return isTouch;
}

uint32_t Slider::duration() {
  if (isTouch) {
    return millis() - touchTimer;
  }
  return touchTimer;
}


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


Slider slider1;
Slider slider2;
uint8_t slider1Pins[NODE_COUNT] = {PIN_PA5, PIN_PA6, PIN_PA7, PIN_PB5, PIN_PB4, PIN_PC1};
uint8_t slider2Pins[NODE_COUNT] = {PIN_PA5, PIN_PA6, PIN_PA7, PIN_PB5, PIN_PB4, PIN_PC1};
uint8_t slider1Driver = 12;
uint8_t slider2Driver = 13;


void setup() {
  // Disable power to next module
  pinMode(PowerSwitchPin, OUTPUT);
  digitalWrite(PowerSwitchPin, HIGH);

  Serial.begin(115200);
  Serial.println("Serial ok");

  // Do address allocation; begin Wire
  delay(100);
  ControlRegisters[0x02] = KC02; // Set module type
  while (!NewAddress) {startupAddressAlloc();}
  Wire.end();
  Serial.print("Beginning wire on 0x");
  Serial.println(NewAddress);
  Wire.begin(NewAddress);

  // Handlers for I2C
  Serial.println("Assigning handlers");
  Wire.onReceive(receiveHandler);
  Wire.onRequest(requestHandler);
  delay(200);

  slider1.begin(slider1Pins, slider1Driver);
  slider2.begin(slider2Pins, slider2Driver);

  Serial.println("Beginning main loop");

}


void loop() {
  us = nus;
  nus = micros();

  digitalWrite(PowerSwitchPin, ControlRegisters[0x03]);

  ptc_process(millis());
  slider1.update();
  slider2.update();

  DeviceRegisters[0x00] = slider1.getPosition();
  DeviceRegisters[0x01] = slider2.getPosition();
  DeviceRegisters[0x02] = slider1.getPositionHysteresis();
  DeviceRegisters[0x03] = slider2.getPositionHysteresis();

  DeviceRegisters[0x04] = slider1.getMagnitude();
  DeviceRegisters[0x05] = slider2.getMagnitude();
  DeviceRegisters[0x06] = slider1.getMagnitudeHysteresis();
  DeviceRegisters[0x07] = slider2.getMagnitudeHysteresis();

  #ifdef DEBUG_SERIAL
  Serial.print(", Slider 1 pos h:");
  Serial.print(DeviceRegisters[0x00]);
  Serial.print(", Slider 2 pos h:");
  Serial.print(DeviceRegisters[0x01]);

  Serial.print(", max:");
  Serial.print(0xFF);
  Serial.print(", min:");
  Serial.println(0);

  #endif
}


void ptc_event_callback(const ptc_cb_event_t eventType, cap_sensor_t* node) {
  #ifdef DEBUG_SERIAL
  if (PTC_CB_EVENT_TOUCH_DETECT == eventType) {
    Serial.print("node touched:");
    Serial.println(ptc_get_node_id(node));
  } else if (PTC_CB_EVENT_TOUCH_RELEASE == eventType) {
    Serial.print("node released:");
    Serial.println(ptc_get_node_id(node));
  } else if (PTC_CB_EVENT_CONV_MUTUAL_CMPL == eventType) {
    // Do more complex things here
  } else if (PTC_CB_EVENT_CONV_CALIB & eventType) {
    if (PTC_CB_EVENT_ERR_CALIB_LOW == eventType) {
      Serial.print("Calib error, Cc too low.");
    } else if (PTC_CB_EVENT_ERR_CALIB_HIGH == eventType) {
      Serial.print("Calib error, Cc too high.");
    } else if (PTC_CB_EVENT_ERR_CALIB_TO == eventType) {
      Serial.print("Calib error, calculation timeout.");
    } else {
      Serial.print("Calib Successful.");
    }
    Serial.print(" Node: ");
    Serial.println(ptc_get_node_id(node));
  }
  #endif
}