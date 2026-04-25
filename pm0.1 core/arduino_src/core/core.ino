// Shepherd Mix CORE Module
// For ESP32-S3

#include <Adafruit_NeoPixel.h>

#include "core_wire_driver.h"
#include "module_driver.h"
#include "screen_update.h"

// I2C ports
#define SCL0 1
#define SDA0 2
#define SCL1 42
#define SDA1 41

// Power output enable pins
#define POWER_LEFT 3
#define POWER_RIGHT 39

// Neopixel
#define NEOPIXEL_PIN 11

// Battery mgmt
#define CG_STAT 40 // Charge status from ic
#define BAT_DET 14 // 1/2 battery voltage


// NeoPixel
Adafruit_NeoPixel pixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);


// Screen
CoreScreen screen;


// MIDI
MidiMgmt mid;


// Wire
CoreWireDriver WireLeft(Wire, mid);
CoreWireDriver WireRight(Wire1, mid);

ModuleDriver DLeft(WireLeft, mid, screen);
ModuleDriver DRight(WireRight, mid, screen);


int timer = 0;


void setup() {
  pinMode(CG_STAT, INPUT_PULLUP);

  // USB serial
  Serial.begin(115200);
  Serial.println("Beginning core module...");

  // NeoPixel
  pixel.begin();
  pixel.clear();
  pixel.setPixelColor(0, pixel.Color(0, 255, 0));
  pixel.show();

  // Screen
  screen.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);

  // Wire
  pinMode(POWER_LEFT, OUTPUT);
  pinMode(POWER_RIGHT, OUTPUT);

  digitalWrite(POWER_LEFT, 0);
  digitalWrite(POWER_RIGHT, 0);

  WireLeft.begin(2, 1, WIRE_CLOCK);
  WireRight.begin(41, 42, WIRE_CLOCK);

  Serial.println("Wire drivers started");

  mid.begin();

  delay(1000);


}

void loop() {
  if (millis() - timer > 1000) {
    timer = millis();

    Serial.println(timer);
    uint8_t success = WireLeft.findNextNewModule();
    if (success) {screen.tft.println(success);}
    success = WireRight.findNextNewModule();
    if (success) {screen.tft.println(success);}
    
  }

  DLeft.updateModules();
  DRight.updateModules();
  
  mid.update();

}
