/**
 * Supernumerary Robotic Finger
 * Version 1.1.5
 * @author Curt Henrichs
 * @date 4-19-19
 *
 * Glove Flex Sensor Driver
 */

//==============================================================================
//  Libraries
//==============================================================================

#include "FlexSensors.h"
#include "HardwareConfig.h"

//==============================================================================
//  Constants and Macro Definitions
//==============================================================================

#define ADC_WORD_SIZE ((float)(1<<12));

/*
const int SENSOR_TO_PIN_MAPPING[__NUM_FLEX_SENSORS__] = {
  FLEX_0_0_PIN, FLEX_0_1_PIN, FLEX_1_0_PIN, FLEX_1_1_PIN, FLEX_2_0_PIN,
  FLEX_2_1_PIN, FLEX_3_0_PIN, FLEX_3_1_PIN, FLEX_4_0_PIN, FLEX_4_1_PIN,
  FLEX_5_0_PIN, FLEX_6_0_PIN
};
*/

//==============================================================================
//  Private Data Members
//==============================================================================

static float _sensors[__NUM_FLEX_SENSORS__];

//==============================================================================
//  Public Function Implementation
//==============================================================================

void fx_init(void) {
  pinMode(MUX_S3_PIN,OUTPUT);
  pinMode(MUX_S2_PIN,OUTPUT);
  pinMode(MUX_S1_PIN,OUTPUT);
  pinMode(MUX_S0_PIN,OUTPUT);
  pinMode(MUX_EN_PIN,OUTPUT);
  pinMode(MUX_VP_PIN,OUTPUT);
  pinMode(MUX_VN_PIN,OUTPUT);

  digitalWrite(MUX_EN_PIN,LOW);
  digitalWrite(MUX_VP_PIN,HIGH);
  digitalWrite(MUX_VN_PIN,LOW);
}

void fx_update(void) {
  for (int i=0; i<__NUM_FLEX_SENSORS__; i++) {
    //_sensors[i] = analogRead(SENSOR_TO_PIN_MAPPING[i]) / ADC_WORD_SIZE;

    digitalWrite(MUX_S3_PIN,((i & 0x08) >> 3));
    digitalWrite(MUX_S2_PIN,((i & 0x04) >> 2));
    digitalWrite(MUX_S1_PIN,((i & 0x02) >> 1));
    digitalWrite(MUX_S0_PIN,((i & 0x01) >> 0));

    analogRead(MUX_SIG_PIN); // give time to settle
    _sensors[i] = analogRead(MUX_SIG_PIN) / ADC_WORD_SIZE;
  }
}

float fx_get(FlexSensorId_t id) {
  if (id < 0 || id >= __NUM_FLEX_SENSORS__) {
    return -1;
  }

  return _sensors[id];
}
