#pragma once

#include <Arduino.h>

#define MIDI_BAUD 31520

#if ARDUINO_USB_MODE
#warning This sketch must be used when USB is in OTG mode
void setup() {}
void loop() {}
#else
#include "USB.h"

#include "esp32-hal-tinyusb.h"

static const char *TAG = "usbdmidi";


// From usb.org MIDI 1.0 specification. This 4 byte structure is the unit
// of transfer for MIDI data over USB.
typedef struct __attribute__((__packed__)) {
  uint8_t code_index_number : 4;
  uint8_t cable_number : 4;
  uint8_t MIDI_0;
  uint8_t MIDI_1;
  uint8_t MIDI_2;
} USB_MIDI_t;


// Basic MIDI Messages
#define NOTE_OFF 0x80
#define NOTE_ON 0x90


class MidiMgmt {
  public:
    uint8_t cable_num = 0; // MIDI jack associated with USB endpoint
    uint8_t channel = 0;   // 0 for channel 1

    static void usbEventCallback(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
    static uint16_t tusb_midi_load_descriptor(uint8_t *dst, uint8_t *itf);
    static void midi_task_read_example();


    MidiMgmt();
    void begin();
    void update();
    void send3BytePacketUSB(uint8_t packet[3]);
    void sendMidiDataUSB(uint8_t event, uint8_t channel, uint8_t midi1, uint8_t midi2);

};

#endif /* ARDUINO_USB_MODE */