#include "midi_mgmt.h"

MidiMgmt::MidiMgmt() {
}

void MidiMgmt::begin() {
  Serial.println("Beginning MIDI...");
  Serial2.begin(MIDI_BAUD, SERIAL_8N1, 43, 44);

  USB.onEvent(usbEventCallback);
  tinyusb_enable_interface(USB_INTERFACE_MIDI, TUD_MIDI_DESC_LEN,
                           tusb_midi_load_descriptor);
  USB.begin();

}

void MidiMgmt::update() {
  if (tud_midi_mounted()) {
    midi_task_read_example();
  }
}

void MidiMgmt::send3BytePacketUSB(uint8_t packet[3]) {
  if (tud_midi_mounted()) {
    tud_midi_stream_write(cable_num, packet, 3);
  }
}

void MidiMgmt::sendMidiDataUSB(uint8_t event, uint8_t channel, uint8_t midi1, uint8_t midi2) {
  uint8_t packet[3] = {event | channel, midi1, midi2};
  Serial.print("Sending midi: ");
  Serial.print(packet[0], HEX); Serial.print(" "); Serial.print(midi1, HEX); Serial.print(" "); Serial.println(midi2, HEX);
  send3BytePacketUSB(packet);
}

void MidiMgmt::usbEventCallback(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data) {
  if (event_base == ARDUINO_USB_EVENTS) {
    arduino_usb_event_data_t *data = (arduino_usb_event_data_t *)event_data;
    switch (event_id) {
    case ARDUINO_USB_STARTED_EVENT:
      Serial.println("USB PLUGGED");
      break;
    case ARDUINO_USB_STOPPED_EVENT:
      Serial.println("USB UNPLUGGED");
      break;
    case ARDUINO_USB_SUSPEND_EVENT:
      Serial.printf("USB SUSPENDED: remote_wakeup_en: %u\n",
                    data->suspend.remote_wakeup_en);
      break;
    case ARDUINO_USB_RESUME_EVENT:
      Serial.println("USB RESUMED");
      break;

    default:
      break;
    }
  }
}

uint16_t MidiMgmt::tusb_midi_load_descriptor(uint8_t *dst, uint8_t *itf) {
  uint8_t str_index = tinyusb_add_string_descriptor("TinyUSB MIDI");
  uint8_t ep_num = tinyusb_get_free_duplex_endpoint();
  TU_VERIFY(ep_num != 0);
  uint8_t descriptor[TUD_MIDI_DESC_LEN] = {
      // Interface number, string index, EP Out & EP In address, EP size
      TUD_MIDI_DESCRIPTOR(*itf, str_index, ep_num, (uint8_t)(0x80 | ep_num),
                          64)};
  *itf += 1;
  memcpy(dst, descriptor, TUD_MIDI_DESC_LEN);
  return TUD_MIDI_DESC_LEN;
}

void MidiMgmt::midi_task_read_example() {
  // The MIDI interface always creates input and output port/jack descriptors
  // regardless of these being used or not. Therefore incoming traffic should be
  // read (possibly just discarded) to avoid the sender blocking in IO
  uint8_t packet[4];
  bool read = false;
    while (tud_midi_available()) {
      read = tud_midi_packet_read(packet);
      if (read) {
        ESP_LOGI(TAG,
                 "Read - Time (ms since boot): %lld, Data: %02hhX %02hhX "
                 "%02hhX %02hhX",
                 esp_timer_get_time(), packet[0], packet[1], packet[2],
                 packet[3]);
        USB_MIDI_t *m = (USB_MIDI_t *)packet;
        Serial.printf(
            "%lld: Cable: %d Code: %01hhX, Data: %02hhX %02hhX %02hhX\n",
            esp_timer_get_time(), m->cable_number, m->code_index_number,
            m->MIDI_0, m->MIDI_1, m->MIDI_2);
      }
    }
}