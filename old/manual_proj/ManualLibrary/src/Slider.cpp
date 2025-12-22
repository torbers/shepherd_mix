#include "Slider.h"

Slider::~Slider() {
    for (uint8_t i = 0; i < count; ++i) {
        delete sensors[i];
    }
    delete[] sensors;
    delete[] pins;
    delete[] baselines;
}

void Slider::begin() {
    for (uint8_t i = 0; i < count; ++i) {
        sensors[i]->begin();
    }
    calibrate();
}

void Slider::calibrate() {
    for (uint8_t i = 0; i < count; ++i) {
        baselines[i] = sensors[i]->measure();
    }
}

uint16_t Slider::read(uint8_t i) {
    if (i >= count) return 0;
    return sensors[i]->measure();
}

int16_t Slider::read_delta(uint8_t i) {
    if (i >= count) return 0;
    return static_cast<int16_t>(sensors[i]->measure()) - static_cast<int16_t>(baselines[i]);
}

uint8_t Slider::getCount() const {
    return count;
}

int Slider::readSlider() {
    uint32_t sumVal = 0;
    uint32_t sumWeighted = 0;

    for (uint8_t i = 0; i < count; i++) {
        int delta = read_delta(i);

        if (delta > NOISE_THRESHOLD) {
            sumVal += delta;
            sumWeighted += delta * (i * POS_MULT);
        }
    }

    if (sumVal > ACTIVATION_THRESHOLD) {
        float centroid = (float)sumWeighted / sumVal;
        int output = (int)(centroid / ((count - 1) * POS_MULT) * SCALE);

        output = map(output, BOTTOMCUT, SCALE - TOPCUT, 0, OUTPUT_RANGE);

        if (output < 0) output = 0;
        else if (output > OUTPUT_RANGE) output = OUTPUT_RANGE;

        return output;
    } else {
        return -1; // No significant touch
    }
}


