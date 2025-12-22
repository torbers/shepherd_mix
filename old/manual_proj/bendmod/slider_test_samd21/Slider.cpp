#include "Slider.h"

/*
Slider::Slider(uint8_t* pins, uint8_t count) {
    this->count = count;
    this->pins = new uint8_t[count];
    sensors = new Adafruit_FreeTouch*[count];

    for (uint8_t i = 0; i < count; i++) {
        this->pins[i] = pins[i];
        sensors[i] = new Adafruit_FreeTouch(pins[i], OVERSAMPLE_4, RESISTOR_0, FREQ_MODE_NONE);
    }
}
*/

Slider::~Slider() {
    for (uint8_t i = 0; i < count; i++) {
        delete sensors[i];
    }
    delete[] sensors;
    delete[] pins;
}

void Slider::begin() {                        // Initialize sensors and set baselines
    for (uint8_t i = 0; i < count; i++) {
      if (!sensors[i]->begin()) {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.println(" failed to start!");
    }
    baseline[i] = sensors[i]->measure();  // Initial baseline
    }
}

uint16_t Slider::read(uint8_t index) {
    if (index >= count) return 0;
    return sensors[index]->measure();
}

uint8_t Slider::getCount() const {
    return count;
}
