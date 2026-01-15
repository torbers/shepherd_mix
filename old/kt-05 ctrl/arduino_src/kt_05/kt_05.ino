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

#define WIRE_CLOCK 1000000

uint8_t BeginningAddress = 0x08;
uint8_t NextNewAddress1 = BeginningAddress + 1;
uint8_t NextNewAddress0 = BeginningAddress + 1;


// Scan for newly connected devices, allocate address
// Takes as arg 0 for Wire or 1 for Wire1
uint8_t findNextNewModule1() {
  Serial.print("Beginning on 0d");
  Serial.println(BeginningAddress);

  Wire1.beginTransmission(BeginningAddress);
  Wire1.write(0xFF);
  Wire1.write(0x01);
  Wire1.write(NextNewAddress1);
  Wire1.endTransmission();

  delay(5);

  Serial.print("Testing 0d");
  Serial.println(NextNewAddress1);

  Wire1.beginTransmission(NextNewAddress1);
  Wire1.write(0xFF);
  Wire1.write(0x00);
  Wire1.write(0x00);
  Wire1.write(NextNewAddress1);
  Wire1.endTransmission();

  Wire1.beginTransmission(NextNewAddress1);
  Wire1.write(0xFF);
  Wire1.endTransmission();

  delay(5);

  uint8_t recieved_addr = 0x00;

  Wire1.requestFrom(NextNewAddress1, 2);

  if (Wire1.available()) {
    Serial.println("W avail...");
    uint8_t wr = Wire1.read();
    Serial.println(wr);

    if (wr == 0x00) {
      Serial.println("Recieved 0x00");
      recieved_addr = Wire1.read();
      Serial.print("Recieved addr 0d");
      Serial.println(recieved_addr);
    }
  }

  if (recieved_addr == NextNewAddress1) {
    NextNewAddress1++;
    return NextNewAddress1 - 1;
  }
  else {return 0x00;}
}

void setup() {

  // USB serial for debug
  // (OK to leave activated for RP2040)
  Serial.begin(115200);
  Serial.println("OK");

  //  I2C SETUP
  // Wire 0 pins: SDA = 4; SCL = 5. Right-side pin connector
  // Wire 1 pins: SDA = 2; SCL = 3. Left-side pin connector
  // Set clock to 1 MHZ
  // Technically out of spec for ATTiny but it works and we need low latency
  // (Change later if it creates too much interference or stops working)
  Wire.setClock(WIRE_CLOCK);
  Wire1.setClock(WIRE_CLOCK);

  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire1.setSDA(2);
  Wire1.setSCL(3);

  Wire.begin();
  Wire1.begin();

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


  // Find first module
  while(!findNextNewModule1()) {
    delay(500);


  }
  Serial.println(NextNewAddress1);

}

void loop() {
  // put your main code here, to run repeatedly:

    Wire1.beginTransmission(0x09);
    Wire1.write(0xC0);
    //Wire1.write(0x20);
    Wire1.endTransmission();
    Wire1.requestFrom(0x09, 0x30);
    //Wire1.requestFrom(0x09, 0x20);
    Serial.print("Changes register:");
    while (Wire1.available()) {Serial.print(" 0x"); Serial.print(Wire1.read(), HEX);}
    Serial.println();

}
