#pragma once

#include <Arduino.h>

#define HALL 0x01
#define MODS 0x02

class Module {
  public:
    // Product name
    uint8_t codename;

    // Addresses are ALWAYS assigned so lowest = closest to control module
    // Module.address - BeginningAddress = distance from ctl
    uint8_t address;

    // Used for deriving velocity
    unsigned long MicrosAtLastRead;
    unsigned long MicrosSinceLastRead;

    // from core
    uint8_t PositionInChain;

    // update when new module is attached; position from left
    uint8_t PositionLeftToRight;

    // Settings
    // Used by all modules
    uint8_t MidiChannel; //  = DEFAULT_MIDI_CHANNEL;

    Module();
};

class Hall: public Module {
  public:
    // Link octave, channel, etc with prev Hall?
    bool LinkedWithPrev; // = true;
    uint8_t OctaveOffset; // = 4;
    uint8_t SemiOffset; // = 4;

    // Trigger NoteOn at bottom of keystroke or top?
    // Bottom (true) for velocity + poly aftertouch, top (false) for aftertouch only
    bool TriggerAtBottomOut; // = true;

    // Do poly aftertouch?
    bool PolyAftertouch; // = true;

    Hall(uint8_t addr);


};

class Mods: public Module {
  public:
      // Used by KC02 only
    // Do bend/mod or CC?
    bool DoBendMod; // = true;

    // CC channels for left and right wheels
    uint8_t CC0; // = 0x10; // General-purpose controllers
    uint8_t CC1; // = 0x11;

    Mods(uint8_t addr);
};
