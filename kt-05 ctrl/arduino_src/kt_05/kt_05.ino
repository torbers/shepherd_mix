/*
KT-05 I2C Control Module
Part of Shepherd System
by Sam Kothe (torbers)

Hardware:
  Microcontroller: RP2040/RP2350
  128x64 OLED Screen: SSD1306
  Switch sensor: SLSS49E
  (Output pin tied to GND via 10k resistor.)

KH-24 linked/unlinked explaination
  0x0A and 0x09 are linked:
  |OCT 4| |OCT 5| |OCT 6| |ctrl  |
  |OCT 1| |OCT 2| |OCT 3| |module|
  0x0B    0x0A    0x09
  (All kh modules have same channel & other settings)

  0x09 is unlinked:
  |OCT 3| |OCT 4| |OCT 2| |ctrl  |
  |OCT 1| |OCT 2| |OCT 1| |module|
  0x0B    0x0A    0x09
  (0x09 may have different channel/settings from 0x0B and 0x0A)

*/

// Libraries
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Hardware definitions
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3c ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define WIRE_CLOCK 1000000

#define WIRE0_SDA 4
#define WIRE0_SCL 5
#define WIRE1_SDA 2
#define WIRE1_SCL 3

#define DEFAULT_MIDI_CHANNEL 0 // MIDI channel 1


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


// Module-to-module comm
uint8_t BeginningAddress = 0x08;
uint8_t NextNewAddress1 = BeginningAddress + 1;
uint8_t NextNewAddress0 = BeginningAddress + 1;


struct Module {
  // Product name
  uint8_t codename;

  // Addresses are ALWAYS assigned so lowest = closest to control module
  // Module.address - BeginningAddress = distance from ctl
  uint8_t address;

  // Used for deriving velocity
  unsigned long MicrosSinceLastRead;

  // update when new module is attached; position from left
  uint8_t PositionLeftToRight;

  // Settings
  // Used by all modules
  uint8_t MidiChannel; //  = DEFAULT_MIDI_CHANNEL;

  // Used by KH-24 only
  // Link octave, channel, etc with prev keyboard?
  // See readme for explaination
  bool LinkedWithPrevKH24; // = true;
  uint8_t OctaveOffset; // = 4;

  // Trigger NoteOn at bottom of keystroke or top?
  // Bottom (true) for velocity + poly aftertouch, top (false) for aftertouch only
  bool TriggerAtBottomOut; // = true;

  // Do poly aftertouch?
  bool PolyAftertouch; // = true;

  // Used by KC02 only
  // Do bend/mod or CC?
  bool DoBendMod; // = true;

  bool LinkedWithPrevKC02; // = true;

  // CC channels for left and right wheels
  uint8_t CC0; // = 0x10; // General-purpose controllers
  uint8_t CC1; // = 0x11;

};


// Module chains to right and left of ctrl module respectively
Module Wire0ModuleChain[16];
Module Wire1ModuleChain[16];


// TODO pass TwoWire Wire or Wire1
// Scan for newly connected devices, allocate address
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
  uint8_t recieved_type;

  Wire1.requestFrom(NextNewAddress1, 3);

  if (Wire1.available()) {
    Serial.println("W avail...");
    uint8_t wr = Wire1.read();
    Serial.println(wr);

    if (wr == 0x00) {
      Serial.println("Recieved 0x00");
      recieved_addr = Wire1.read();
      Serial.print("Recieved addr 0d");
      Serial.println(recieved_addr);

      recieved_type = Wire1.read();
    }
  }

  if (recieved_addr == NextNewAddress1) {
    // connection confirmed - create module

    //todo

    NextNewAddress1++;
    return NextNewAddress1 - 1;
  }
  else {return 0x00;}
}


void keyChangesWire1KH24(uint8_t address) {
  // Request key changes...

  Wire1.beginTransmission(address);
  Wire1.write(0xC0);
  Wire1.endTransmission();
  Wire1.requestFrom(address, 0x20);
  
  uint8_t KeyNumber;
  uint8_t CurrentPosition;
  uint8_t OldPosition;
  uint8_t EventType;
  uint8_t StateCode;
  uint8_t NewState;
  uint8_t OldState;
  
  while (Wire1.available()) {
    KeyNumber = Wire1.read() - 0x80;
    StateCode = Wire1.read();
    CurrentPosition = Wire1.read();
    OldPosition = Wire1.read();

    OldState = StateCode >> 4;
    NewState = StateCode & 0b00000011;

    
  }
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

  Wire.setSDA(WIRE0_SDA);
  Wire.setSCL(WIRE0_SCL);
  Wire1.setSDA(WIRE1_SDA);
  Wire1.setSCL(WIRE1_SCL);

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
    Wire1.endTransmission();
    Wire1.requestFrom(0x09, 0x20);
    Serial.print("Changes register:");
    while (Wire1.available()) {Serial.print(" 0x"); Serial.print(Wire1.read(), HEX);}
    Serial.println();

}
