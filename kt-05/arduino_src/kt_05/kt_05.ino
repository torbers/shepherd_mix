/*
KT-05 I2C Control Module
Part of Shepherd System
by Sam Kothe (torbers)

Hardware:
  Microcontroller: RP2040/RP2350
  128x64 OLED Screen: SSD1306
  Switch sensor: SLSS49E
  (Output pin tied to GND via 10k resistor.)

*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3c ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


void setup() {

  // USB serial for debug
  // (OK to leave activated for RP2040)
  Serial.begin(115200);

  //  I2C SETUP
  // Wire 0 pins: SDA = 4; SCL = 5. Right-side pin connector
  // Wire 1 pins: SDA = 2; SCL = 3. Left-side pin connector
  // Set clock to 1 MHZ
  // Technically out of spec for ATTiny but it works and we need low latency
  // (Change later if it creates too much interference or stops working)
  Wire.setClock(1000000);
  Wire1.setClock(1000000);

  Wire.begin()
  Wire1.begin()

  delay(100);

  // DISPLAY SETUP
  // Display is on Wire 0
  // SDA = 4; SCL = 5
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(100);
  display.clearDisplay();

}

void loop() {
  // put your main code here, to run repeatedly:

}
