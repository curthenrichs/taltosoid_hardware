/**
 * Supernumerary Robotic Finger
 * Version 1.1.5
 * @author Curt Henrichs
 * @date 4-19-19
 *
 * Glove Flex Sensor Driver
 */

#ifndef ANALOG_MUX_H
#define ANALOG_MUX_H

//==============================================================================
//  Constants and Macro Definitions
//==============================================================================

typedef enum AMuxSensorId {

  // Available sensors
  FLEX_0 = 0,
  FLEX_1 = 1,

  // Private value, used to define range of joints
  __NUM_SENSORS__
} AMuxSensorId_t;

//==============================================================================
//  Public Function Prototypes
//==============================================================================

void amux_init(void);
void amux_update(void);
float amux_get(AMuxSensorId_t id);

#endif
