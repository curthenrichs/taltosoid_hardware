/**
 * Supernumerary Robotic Finger
 * Version 1.1.5
 * @author Curt Henrichs
 * @date 4-19-19
 *
 * Glove Flex Sensor Driver
 */

#ifndef FLEX_SENSOR_DRIVER_H
#define FLEX_SENSOR_DRIVER_H

//==============================================================================
//  Libraries
//==============================================================================

//==============================================================================
//  Constants and Macro Definitions
//==============================================================================

typedef enum FlexSensorId {

  // Available sensors
  FLEX_0_0,
  FLEX_0_1,
  FLEX_1_0,
  FLEX_1_1,
  FLEX_2_0,
  FLEX_2_1,
  FLEX_3_0,
  FLEX_3_1,
  FLEX_4_0,
  FLEX_4_1,
  FLEX_5_0,
  FLEX_6_0,

  // Private value, used to define range of joints
  __NUM_FLEX_SENSORS__
} FlexSensorId_t;

//==============================================================================
//  Public Function Prototypes
//==============================================================================

void fx_init(void);
void fx_update(void);
float fx_get(FlexSensorId_t id);

#endif
