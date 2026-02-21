// Shepherd Mix CORE Module
// For ESP32-S3

#include <Adafruit_NeoPixel.h>

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


void setup() {
  pinMode(POWER_LEFT, OUTPUT);
  pinMode(POWER_RIGHT, OUTPUT);
  pinMode(CG_STAT, INPUT_PULLUP);

  // USB serial
  Serial.begin(115200);
  Serial.println("Beginning core module...");

  // NeoPixel
  pixel.begin();
  pixel.clear();
  pixel.setPixelColor(0, pixel.Color(255, 255, 255));
  pixel.show();

  // Screen
  screen.begin();





}

void loop() {
  // put your main code here, to run repeatedly:

}
