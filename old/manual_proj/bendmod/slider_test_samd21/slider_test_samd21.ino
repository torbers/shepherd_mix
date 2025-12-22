
#include <Adafruit_FreeTouch.h>
#include <MIDIUSB.h>  // Native USB MIDI

// Touch slider using interpolated centroid method

Adafruit_FreeTouch sensors[] = {
  Adafruit_FreeTouch(A0, OVERSAMPLE_4, RESISTOR_0, FREQ_MODE_NONE),  // Y0
  Adafruit_FreeTouch(A1, OVERSAMPLE_4, RESISTOR_0, FREQ_MODE_NONE),  // Y14
  Adafruit_FreeTouch(A2, OVERSAMPLE_4, RESISTOR_0, FREQ_MODE_NONE),  // Y15
  Adafruit_FreeTouch(A3, OVERSAMPLE_4, RESISTOR_0, FREQ_MODE_NONE),  // Y2
  Adafruit_FreeTouch(A4, OVERSAMPLE_4, RESISTOR_0, FREQ_MODE_NONE),  // Y3
  Adafruit_FreeTouch(A5, OVERSAMPLE_4, RESISTOR_0, FREQ_MODE_NONE)   // Y8
};

const int N = sizeof(sensors) / sizeof(sensors[0]);
uint16_t baseline[N];
int lastMidiValue = -1;  // to prevent repeated MIDI sends

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Initialize sensors and capture baseline
  for (int i = 0; i < N; i++) {
    if (!sensors[i].begin()) {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.println(" failed to start!");
      while (1);
    }
    baseline[i] = sensors[i].measure();  // Initial baseline
  }

  Serial.println("Touch slider initialized.");
}

int bottomcut = 40;
int topcut = 60;

int SCALE = 255;
const int POS_MULT = 1000; // scaling for float precision
int OUTPUT_RANGE = 127;


void loop() {
  uint32_t sumVal = 0;
  uint32_t sumWeighted = 0;

  for (int i = 0; i < N; i++) {
    uint16_t val = sensors[i].measure();

    int delta = (int)val - (int)baseline[i];
    if (delta > 10) {  // Reject small noise (adjustable threshold)
      sumVal += delta;
      sumWeighted += delta * (i * POS_MULT);
    }
  }

  Serial.println();

  if (sumVal > 300) {
    float centroid = (float)sumWeighted / sumVal; // Position in [0, (N-1)*POS_MULT]
    int output = (int)(centroid / ((N - 1) * POS_MULT) * SCALE);


    output = map(output, bottomcut, SCALE - topcut, 0, OUTPUT_RANGE);


    if (output < 0) {output = 0;}
    else if (output > OUTPUT_RANGE) {output = OUTPUT_RANGE;}

    Serial.print("Max:");
    Serial.print(127);
    Serial.print(" ");

    Serial.print("Min:");
    Serial.print(0);
    Serial.print(" ");

    //Serial.print("Sum:");
    //Serial.print(sumVal);
    //Serial.print(" ");

    Serial.print("Slider value:");
    Serial.println(output);

    // Send MIDI only if value has changed
    if (output != lastMidiValue) {
      sendModWheel(output);
      lastMidiValue = output;
    }

  } else {
    //Serial.println("No touch");
  }

  delay(1);
}

void sendModWheel(uint8_t value) {
  midiEventPacket_t event = {0x0B, 0xB0 | 0x00, 0x01, value};  // CC on channel 1, controller 1 (mod wheel)
  MidiUSB.sendMIDI(event);
  MidiUSB.flush();
}
