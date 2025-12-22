#include <ManualLibrary.h>

uint8_t touchPins[] = {1, 3, 4};
int ACTIVATION_THRESHOLD=100;
Slider slider(touchPins);

// Touch slider using interpolated centroid method

void setup() {
  Serial.begin(115200);
  delay(1000);

  slider.begin();
  

}

void loop() {
  Serial.println(slider.readSlider());
  digitalWrite(2, LOW);
  delay(1);
  digitalWrite(2, HIGH);
  delay(1);
}

