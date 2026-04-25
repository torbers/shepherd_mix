#pragma once

#include <Arduino.h>

#include "modules.h"
#include "midi_mgmt.h"
#include "core_wire_driver.h"

#include "screen_update.h"

#include <map>

#define CHANGES_LENGTH 0x20


class ModuleDriver {
  public:
    CoreWireDriver* WireDriver;

    MidiMgmt* midi;
    CoreScreen* screen;

    uint8_t key_map[0x20] = {0xFF, 20, 18, 22, 0xFF, 13, 15, 0xFF, 21, 19, 17, 23, 16, 12, 14, 0xFF, 0xFF, 8, 6, 10, 0xFF, 1, 3, 0xFF, 9, 7, 5, 11, 4, 0, 2, 0xFF};
    uint8_t rvs_key_map[24] = {29, 21, 30, 22, 28, 26, 17, 25, 17, 24, 19, 27, 13, 5, 14, 6, 12, 10, 2, 9, 1, 8, 3, 11};

    uint8_t updateModules();

    uint8_t* getChangesHall(uint8_t addr);
    uint8_t* getValuesHall(uint8_t addr);
    uint8_t* getCalibValuesHall(uint8_t addr);
    uint8_t handleChangesRegisterHall(uint8_t *changes_register);
    uint8_t* keyToNoteOctHall(uint8_t key);
    uint8_t sortKeys(uint8_t key_number);
    uint8_t doBottomOutCalibrationRoutine(uint8_t addr);

    ModuleDriver(CoreWireDriver& wire_driver_arg, MidiMgmt& midi_mgmt, CoreScreen& screen_arg);
};