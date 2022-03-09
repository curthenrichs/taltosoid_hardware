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

#include "AnalogMux.h"
#include "HardwareConfig.h"

//==============================================================================
//  Constants and Macro Definitions
//==============================================================================

#define ADC_WORD_SIZE ((float)(1<<12));

//==============================================================================
//  Private Data Members
//==============================================================================

static float _sensors[__NUM_FLEX_SENSORS__];

//==============================================================================
//  Public Function Implementation
//==============================================================================

void amux_init(void) {
  pinMode(MUX_S3_PIN,OUTPUT);
  pinMode(MUX_S2_PIN,OUTPUT);
  pinMode(MUX_S1_PIN,OUTPUT);
  pinMode(MUX_S0_PIN,OUTPUT);
  pinMode(MUX_EN_PIN,OUTPUT);
  pinMode(MUX_VP_PIN,OUTPUT);
  pinMode(MUX_VN_PIN,OUTPUT);

  digitalWrite(MUX_EN_PIN,LOW); // Active low enable
  digitalWrite(MUX_VP_PIN,HIGH); // High voltage reference
  digitalWrite(MUX_VN_PIN,LOW);  // Low voltage reference
}

void amux_update(void) {
  for (int i=0; i<__NUM_SENSORS__; i++) {
    digitalWrite(MUX_S3_PIN,((i & 0x08) >> 3));
    digitalWrite(MUX_S2_PIN,((i & 0x04) >> 2));
    digitalWrite(MUX_S1_PIN,((i & 0x02) >> 1));
    digitalWrite(MUX_S0_PIN,((i & 0x01) >> 0));

    delayMicroseconds(10);// give time to settle
    analogRead(MUX_SIG_PIN); 
    _sensors[i] = analogRead(MUX_SIG_PIN) / ADC_WORD_SIZE;
  }
}

float amux_get(AMuxSensorId_t id) {
  if (id < 0 || id >= __NUM_SENSORS__) {
    return -1.0f;
  }

  return _sensors[id];
}
