// {D8, D3, D4} = x in bootmode (x,y)'
// D8 = low, D4 = high
// D3 = low if UART, high if flash


#include <ArduinoJson.h>


#define FLEX_INDEX_PIN A0

static StaticJsonDocument<200> root;

void setup(void) {
  Serial.begin(74880);
}

void loop(void) {

  // Read flex sensors
  root.clear();
  root["flex"] = analogRead(FLEX_INDEX_PIN);
  //serializeJson(root,Serial);
  Serial.println((int)root["flex"]);
  delay(100);
}
