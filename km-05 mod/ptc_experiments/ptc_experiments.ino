#include <ptc_touch.h>

#define NODE_COUNT 4
cap_sensor_t nodes[NODE_COUNT];

void setup() {
  Serial.begin(115200);

  ptc_add_selfcap_node(&nodes[0], PIN_TO_PTC(PIN_PA5), PIN_TO_PTC(PIN_PA4));    
  ptc_add_selfcap_node(&nodes[1], PIN_TO_PTC(PIN_PA6), PIN_TO_PTC(PIN_PA4));
  ptc_add_selfcap_node(&nodes[2], PIN_TO_PTC(PIN_PA7), PIN_TO_PTC(PIN_PA4));
  ptc_add_selfcap_node(&nodes[3], PIN_TO_PTC(PIN_PB5), PIN_TO_PTC(PIN_PA4));

  for (int i = 0; i < NODE_COUNT; i++) {
    ptc_node_set_gain(&nodes[i], 1, 4);
    delay(100);
    nodes[i].reference = ptc_get_node_sensor_value(&nodes[i]);

  }

}

void loop() {
  ptc_process(millis());
  for (int i = 0; i < NODE_COUNT; i++) {
    Serial.print(i);
    Serial.print(":");
    Serial.print(ptc_get_node_delta(&nodes[i]));
    Serial.print(", ");
  }
  Serial.print("max:");
  Serial.print(512);
  Serial.print(", min:");
  Serial.println(-512);
}

void ptc_event_callback(const ptc_cb_event_t eventType, cap_sensor_t* node) {
  if (PTC_CB_EVENT_TOUCH_DETECT == eventType) {
    Serial.print("node touched:");
    Serial.println(ptc_get_node_id(node));
  } else if (PTC_CB_EVENT_TOUCH_RELEASE == eventType) {
    Serial.print("node released:");
    Serial.println(ptc_get_node_id(node));
  } else if (PTC_CB_EVENT_CONV_MUTUAL_CMPL == eventType) {
    // Do more complex things here
  } else if (PTC_CB_EVENT_CONV_CALIB & eventType) {
    if (PTC_CB_EVENT_ERR_CALIB_LOW == eventType) {
      Serial.print("Calib error, Cc too low.");
    } else if (PTC_CB_EVENT_ERR_CALIB_HIGH == eventType) {
      Serial.print("Calib error, Cc too high.");
    } else if (PTC_CB_EVENT_ERR_CALIB_TO == eventType) {
      Serial.print("Calib error, calculation timeout.");
    } else {
      Serial.print("Calib Successful.");
    }
    Serial.print(" Node: ");
    Serial.println(ptc_get_node_id(node));
  }
}