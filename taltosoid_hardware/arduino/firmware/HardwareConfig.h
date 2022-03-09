/**
 * Supernumerary Robotic Finger
 * Version 1.1.5
 * @author Curt Henrichs
 * @date 4-19-19
 *
 * Hardware Configuration
 */

#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

//==============================================================================
//  Control Flags
//==============================================================================

#define DEBUG_MODE (false)

//==============================================================================
//  Libraries
//==============================================================================

#include <Arduino.h>
#include <Wire.h>

#if !DEBUG_MODE
#include <BluetoothSerial.h>
#endif

//==============================================================================
//  Device Pinout
//==============================================================================

#define MUX_SIG_PIN                                 (33)
#define MUX_S3_PIN                                  (25)
#define MUX_S2_PIN                                  (26)
#define MUX_S1_PIN                                  (27)
#define MUX_S0_PIN                                  (14)
#define MUX_EN_PIN                                  (12)
#define MUX_VP_PIN                                  (13)
#define MUX_VN_PIN                                  (0)

#define I2C_SCL_PIN                                 (22)
#define I2C_SDA_PIN                                 (21)

//==============================================================================
//  Constants and Macro Definitions
//==============================================================================

#if DEBUG_MODE
#define DEBUG_SERIAL                                (Serial)
#define DEBUG_SERIAL_BAUD                           (115200)
#endif

extern const char* VERSION_STR;

#if !DEBUG_MODE
extern BluetoothSerial SerialBT;
extern const char* BLUETOOTH_DEV_NAME;
#endif

#if DEBUG_MODE
#define API_SERIAL                                  DEBUG_SERIAL
#else
#define API_SERIAL                                  SerialBT
#endif

//==============================================================================
//  I2C Addresses
//==============================================================================

#define SERVO_DRIVER_ADDRESS (0x40)

//==============================================================================
//  Public Function Prototypes
//==============================================================================

void hw_init(void);

#endif
