#include <ptc_touch.h>
#include "Hysteresis.h"

#define NODE_COUNT 6


// slider logic
// adapted from https://github.com/TinyCircuits/TinyCircuits-ATtiny841-Library/tree/master

class Slider {
  public:
    //Slider();
    uint8_t begin(uint8_t *sensors, uint8_t driver);
    int16_t capTouchRead(uint8_t pin);
    int16_t getPosition(int range=0xFF);
    int16_t getMaxReading();
    int16_t getMagnitude();
    int16_t getPositionHysteresis();
    int16_t getMagnitudeHysteresis();
    
    uint32_t duration();
    bool update();
    bool isTouched();
    
    static const uint8_t numSensors = NODE_COUNT;
    uint8_t capTouchPins[numSensors] = {};
    uint8_t driver_pin = 0;

    int16_t capTouchCal[numSensors];
    int16_t capTouchCurrent[numSensors];
    uint8_t overCalCount[numSensors];

    int16_t currentMaxReading= 0;
    uint8_t primary = 0;
    bool isTouch = 0;
    uint8_t initialTouchPos = 0;
    uint8_t finalTouchPos = 0;
    uint8_t currentTouchedPositions = 0;
    uint8_t allTouchedPositions = 0;
    uint32_t touchTimer = 0;

    cap_sensor_t s_nodes[numSensors];

    Hysteresis <int16_t> positionHysteresis = Hysteresis <int16_t> (4);
    Hysteresis <int16_t> magnitudeHysteresis = Hysteresis <int16_t> (4);
};


uint8_t Slider::begin(uint8_t sensors[numSensors], uint8_t driver) {

  driver_pin = driver;
  
  for (uint8_t p = 0; p < numSensors; p++) {
    Serial.print("Sensor no.: ");
    Serial.println(p);
    capTouchPins[p] = sensors[p];
    Serial.print("goes to pin no.: ");
    Serial.println(capTouchPins[p]);
  }

  for (uint8_t pin = 0; pin < numSensors; pin++) {
    ptc_add_selfcap_node(&s_nodes[pin], PIN_TO_PTC(capTouchPins[pin]), PIN_TO_PTC(driver_pin));
    delay(100);
  }

  ptc_process(millis());

  for (uint8_t pin = 0; pin < numSensors; pin++) {
    capTouchCal[pin] = capTouchRead(pin);
    overCalCount[pin] = 0;
    delay(100);
  }

  for (uint8_t p = 0; p < numSensors; p++) {
    Serial.print("Sensor read: ");
    Serial.println(capTouchCal[p]);
  }
  
  return 0;
}

int16_t Slider::capTouchRead(uint8_t pin) {
  //return ptc_get_node_sensor_value(&s_nodes[capTouchPins[pin] - 1]);
  return ptc_get_node_sensor_value(&s_nodes[pin]) - 512;
}

int16_t Slider::getPosition(int range) {
  float position = primary;
  if (primary < numSensors-1)
    position += (pow((float)capTouchCurrent[primary + 1] / (float)getMaxReading(),0.45)) / 2.0;
  if (primary > 0)
    position -= (pow((float)capTouchCurrent[primary - 1] / (float)getMaxReading(),0.45)) / 2.0;
  
  position = position * (float)range / float(numSensors-1);
  return position;
}

int16_t Slider::getMaxReading() {
  return currentMaxReading;
}

int16_t Slider::getMagnitude() {
  uint8_t j;
  long magnitude = 0;
  for (j = 0; j < 6; j++) {
    magnitude += pow(capTouchCurrent[j], 2);
  }
  return sqrt(magnitude);
}

int16_t Slider::getPositionHysteresis() {
  Serial.println(getPosition());
  int16_t ph = positionHysteresis.add(getPosition());
  Serial.println(ph);
  return ph;
}

int16_t Slider::getMagnitudeHysteresis() {
  return magnitudeHysteresis.add(getMagnitude());
}

bool Slider::update() {
  uint8_t j;
  currentMaxReading=0;
  for (j = 0; j < numSensors; j++) {
    int val = capTouchRead(j);
    if (val < capTouchCal[j]) {
      capTouchCal[j] = val;
    } else if (val > capTouchCal[j] + 1) {
      overCalCount[j]++;
      if (overCalCount[j] > 3) {
        overCalCount[j] = 0;
        capTouchCal[j] += 1;
      } else {
        //overCalCount[j]=0;
      }
    }
    capTouchCurrent[j] = val  - capTouchCal[j];  
    if (capTouchCurrent[j] > getMaxReading()) {
      currentMaxReading = capTouchCurrent[j];
      primary = j;
    }
  }

  bool completedTouch = false;

  if (currentMaxReading > 255) {
    if (!isTouch) {
      isTouch = true;
      touchTimer = millis();
      initialTouchPos = primary;
      finalTouchPos = primary;
      currentTouchedPositions = 0;
      allTouchedPositions = 0;
    } else {
      finalTouchPos = primary;
      currentTouchedPositions = 0;
      for (j = 0; j < numSensors; j++) {
        if (capTouchCurrent[j] > 64)
          currentTouchedPositions |= (1 << j);
        allTouchedPositions |= (1 << j);
      }
    }
  } else {
    if (isTouch) {
      isTouch = false;
      touchTimer = millis() - touchTimer;
      completedTouch = true;
    }
  }

  return completedTouch;
}

bool Slider::isTouched() {
  return isTouch;
}

uint32_t Slider::duration() {
  if (isTouch) {
    return millis() - touchTimer;
  }
  return touchTimer;
}


Slider slider1;
Slider slider2;
uint8_t ps[NODE_COUNT] = {PIN_PA5, PIN_PA6, PIN_PA7, PIN_PB5, PIN_PB4, PIN_PC1};


void setup() {
  Serial.begin(115200);

  delay(200);
  slider1.begin(ps, PIN_PA4);

/*
  ptc_add_selfcap_node(&nodes[0], PIN_TO_PTC(PIN_PA5), PIN_TO_PTC(PIN_PA4));    
  ptc_add_selfcap_node(&nodes[1], PIN_TO_PTC(PIN_PA6), PIN_TO_PTC(PIN_PA4));
  ptc_add_selfcap_node(&nodes[2], PIN_TO_PTC(PIN_PA7), PIN_TO_PTC(PIN_PA4));
  ptc_add_selfcap_node(&nodes[3], PIN_TO_PTC(PIN_PB5), PIN_TO_PTC(PIN_PA4));
  ptc_add_selfcap_node(&nodes[4], PIN_TO_PTC(PIN_PB4), PIN_TO_PTC(PIN_PA4));
  ptc_add_selfcap_node(&nodes[5], PIN_TO_PTC(PIN_PC1), PIN_TO_PTC(PIN_PA4));

  delay(200);

  for (int i = 0; i < NODE_COUNT; i++) {
    ptc_node_set_gain(&nodes[i], 1, 4);
    delay(100);
    nodes[i].reference = ptc_get_node_sensor_value(&nodes[i]);

  }
*/

}


void loop() {
  ptc_process(millis());
  slider1.update();
  int16_t position = slider1.getPositionHysteresis();


  for (uint8_t i = 6; i < NODE_COUNT; i++) {
    int nodeivalue = slider1.capTouchRead(i);
    /*
    Serial.print(i);
    Serial.print(":");
    Serial.print(nodeivalue);
    Serial.print(", ");
    */
    
  }
  //if (slider1.isTouched()) {
  //  Serial.print("pos:");
  //  Serial.print(position);
  //}
  Serial.print(", max:");
  Serial.print(0xFF);
  Serial.print(", min:");
  Serial.println(0);

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

