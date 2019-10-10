/**
 * Supernumerary Robotic Finger
 * Version 1.1.5
 * @author Curt Henrichs
 * @date 4-19-19
 *
 * Hardware Configuration
 */

//==============================================================================
//  Libraries
//==============================================================================

#include "HardwareConfig.h"

//==============================================================================
//  Constants and Macro Definitions
//==============================================================================

const char* VERSION_STR = "1.1.5";
const char* BLUETOOTH_DEV_STR = "TALTOSIOD-API";

//==============================================================================
//  Public Data Members
//==============================================================================

BluetoothSerial SerialBT;

//==============================================================================
//  Public Function Implementation
//==============================================================================

void hw_init(void) {
  Wire.begin();

  DEBUG_SERIAL.setRxBufferSize(512);
  DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUD);

  SerialBT.begin(BLUETOOTH_DEV_STR);
}
