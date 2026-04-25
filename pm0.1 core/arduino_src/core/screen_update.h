#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

// TFT screen definitions
#define TFT_CS        18
#define TFT_RST        14 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC         15

#define TFT_MOSI 16 // Data out
#define TFT_SCLK 17  // Clock out
#define TFT_MISO -1  // Not needed

#define SPI_SPEED 40000000


class CoreScreen {
  public:
    SPIClass hspi = SPIClass(HSPI);
    Adafruit_ST7789 tft = Adafruit_ST7789(&hspi, TFT_CS, TFT_DC, TFT_RST);

    bool begin(uint8_t tft_sclk, uint8_t tft_miso, uint8_t tft_mosi, uint8_t tft_cs);
    void update();
    void print(String str);

};

