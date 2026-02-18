// Shepherd Mix CORE Module
// For ESP32-S3

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_NeoPixel.h>

#include <SPI.h>
#include <Wire.h>


// TFT screen definitions
#define TFT_CS        18
#define TFT_RST        14 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC         6 // 15

#define TFT_MOSI 12 // 16 // Data out
#define TFT_SCLK 17  // Clock out
#define TFT_MISO -1  // Not needed

#define SPI_SPEED 40000000

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

// Charge status
#define CG_STAT 40

// 1/2 battery voltage
#define BAT_DET 14

// TFT screen objects
SPIClass hspi = SPIClass(HSPI);
Adafruit_ST7789 tft = Adafruit_ST7789(&hspi, TFT_CS, TFT_DC, TFT_RST);

// NeoPixel
Adafruit_NeoPixel pixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);


void setup() {
  pinMode(POWER_LEFT, OUTPUT);
  pinMode(POWER_RIGHT, OUTPUT);
  pinMode(CG_STAT, INPUT_PULLUP);

  // USB serial
  Serial.begin(115200);
  Serial.println("Beginning core module...");
  
  // Setup TFT screen
  hspi.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);
  tft.init(135, 240);
  tft.setSPISpeed(SPI_SPEED);

  // NeoPixel
  pixel.begin();
  pixel.clear();
  pixel.setPixelColor(0, pixel.Color(255, 255, 255));
  pixel.show();





}

void loop() {
  // put your main code here, to run repeatedly:

}
