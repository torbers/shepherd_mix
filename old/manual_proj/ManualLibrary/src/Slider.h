#ifndef SLIDER_H
#define SLIDER_H

#include <Adafruit_FreeTouch.h>

class Slider {
public:
    template <size_t N>
    Slider(uint8_t (&pins)[N],
           oversample_t oversample = OVERSAMPLE_4,
           series_resistor_t resistor = RESISTOR_0,
           freq_mode_t freq = FREQ_MODE_NONE,
           int BOTTOMCUT = 40,
           int TOPCUT = 60,
           int SCALE = 255,
           int POS_MULT = 1000,
           int OUTPUT_RANGE = 127,
           int NOISE_THRESHOLD = 10,
           int ACTIVATION_THRESHOLD = 300)
        : count(N),
          oversample(oversample), resistor(resistor), freq(freq),
          BOTTOMCUT(BOTTOMCUT), TOPCUT(TOPCUT), SCALE(SCALE),
          POS_MULT(POS_MULT), OUTPUT_RANGE(OUTPUT_RANGE),
          NOISE_THRESHOLD(NOISE_THRESHOLD), ACTIVATION_THRESHOLD(ACTIVATION_THRESHOLD)
    {
        this->pins = new uint8_t[N];
        this->sensors = new Adafruit_FreeTouch*[N];
        this->baselines = new uint16_t[N];

        for (size_t i = 0; i < N; ++i) {
            this->pins[i] = pins[i];
            this->sensors[i] = new Adafruit_FreeTouch(pins[i], oversample, resistor, freq);
            this->baselines[i] = 0;
        }
    }

    ~Slider();

    void begin();
    void calibrate();
    uint16_t read(uint8_t index);
    int16_t read_delta(uint8_t index);
    uint8_t getCount() const;
    int readSlider(); // ← centroid position
    
    // Setters for updating parameters at runtime
    void setBottomCut(int value) { BOTTOMCUT = value; }
    void setTopCut(int value) { TOPCUT = value; }
    void setScale(int value) { SCALE = value; }
    void setPosMult(int value) { POS_MULT = value; }
    void setOutputRange(int value) { OUTPUT_RANGE = value; }
    void setNoiseThreshold(int value) { NOISE_THRESHOLD = value; }
    void setActivationThreshold(int value) { ACTIVATION_THRESHOLD = value; }

    // Optional: set all in one call
    void setParams(int bottomcut, int topcut, int scale, int posMult, int outputRange,
                   int noiseThreshold, int activationThreshold) {
        BOTTOMCUT = bottomcut;
        TOPCUT = topcut;
        SCALE = scale;
        POS_MULT = posMult;
        OUTPUT_RANGE = outputRange;
        NOISE_THRESHOLD = noiseThreshold;
        ACTIVATION_THRESHOLD = activationThreshold;
    }


private:
    Adafruit_FreeTouch** sensors;
    uint8_t* pins;
    uint16_t* baselines;
    uint8_t count;

    // Config
    oversample_t oversample;
    series_resistor_t resistor;
    freq_mode_t freq;

    // Centroid and filtering parameters
    int BOTTOMCUT;
    int TOPCUT;
    int SCALE;
    int POS_MULT;
    int OUTPUT_RANGE;
    int NOISE_THRESHOLD;
    int ACTIVATION_THRESHOLD;
};

#endif
