#pragma once

#include <Adafruit_FreeTouch.h>

class Slider {
public:
    template <size_t N>
    Slider(uint8_t (&pins)[N]) : count(N) {
        this->pins = new uint8_t[N];
        sensors = new Adafruit_FreeTouch*[N];
        for (size_t i = 0; i < N; ++i) {
            this->pins[i] = pins[i];
            sensors[i] = new Adafruit_FreeTouch(pins[i], OVERSAMPLE_4, RESISTOR_0, FREQ_MODE_NONE);
        }
    }

    ~Slider();

    void begin();
    uint16_t read(uint8_t index);
    uint8_t getCount() const;

private:
    Adafruit_FreeTouch** sensors;
    uint8_t* pins;
    uint8_t count;
    int* baseline;
};
