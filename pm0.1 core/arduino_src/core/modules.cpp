
#include <cstdint>
#include "modules.h"

Module::Module() {codename = 0xFF;}

Hall::Hall(uint8_t addr) {
  codename = HALL;
  address = addr;

}

Mods::Mods(uint8_t addr) {
  codename = MODS;
  address = addr;
  CC0 = 0x10;
  CC1 = 0x11;

}