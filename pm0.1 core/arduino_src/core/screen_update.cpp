
#include "screen_update.h"

bool CoreScreen::begin(uint8_t tft_sclk=TFT_SCLK, uint8_t tft_miso=TFT_MISO, uint8_t tft_mosi=TFT_MOSI, uint8_t tft_cs=TFT_CS) {
    // Setup TFT screen
  bool ok = hspi.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);
  tft.init(135, 240);
  tft.fillScreen(0x07E0);
  tft.setCursor(0, 0);
  tft.setTextColor(0x000000);
  tft.setTextSize(2);
  tft.setTextWrap(true);
  tft.setSPISpeed(SPI_SPEED);
  return ok;
}

void CoreScreen::update() {
  //return;
}
