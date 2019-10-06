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
//  Libraries
//==============================================================================

#include <Arduino.h>
#include <Wire.h>

#include "BluetoothSerial.h"

//==============================================================================
//  Control Flags
//==============================================================================

#define DEBUG_MODE (false)

//==============================================================================
//  Device Pinout
//==============================================================================

/*
#define FLEX_0_0_PIN (13)
#define FLEX_0_1_PIN (12)
#define FLEX_1_0_PIN (14)
#define FLEX_1_1_PIN (27)
#define FLEX_2_0_PIN (26)
#define FLEX_2_1_PIN (25)
#define FLEX_3_0_PIN (33)
#define FLEX_3_1_PIN (32)
#define FLEX_4_0_PIN (15)
#define FLEX_4_1_PIN (0)
#define FLEX_5_0_PIN (4)
#define FLEX_6_0_PIN (2)
*/

#define MUX_SIG_PIN (33)
#define MUX_S3_PIN (25)
#define MUX_S2_PIN (26)
#define MUX_S1_PIN (27)
#define MUX_S0_PIN (14)
#define MUX_EN_PIN (12)
#define MUX_VP_PIN (13)
#define MUX_VN_PIN (0)

#define I2C_SCL_PIN (22)
#define I2C_SDA_PIN (21)

//==============================================================================
//  Constants and Macro Definitions
//==============================================================================

#define DEBUG_SERIAL (Serial)
#define DEBUG_SERIAL_BAUD (115200)

extern const char* VERSION_STR;

extern BluetoothSerial SerialBT;
extern const char* BLUETOOTH_DEV_NAME;

#if DEBUG_MODE
#define API_SERIAL DEBUG_SERIAL
#else
#define API_SERIAL SerialBT
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
