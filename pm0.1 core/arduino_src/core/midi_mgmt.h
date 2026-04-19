#pragma once

#include <Arduino.h>

class MidiMgmt {
  public:
    uint8_t sendMidi(uint8_t* message);
};