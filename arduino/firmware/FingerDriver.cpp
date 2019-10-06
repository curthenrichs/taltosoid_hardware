/**
 * Supernumerary Robotic Finger
 * Version 1.1.5
 * @author Curt Henrichs
 * @date 4-19-19
 *
 * Robotic Finger Joint Driver
 */

//==============================================================================
//  Libraries
//==============================================================================

#include "FingerDriver.h"
#include "HardwareConfig.h"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//==============================================================================
//  Constants and Macro Definitions
//==============================================================================

#define SERVO_MIN  (150)
#define SERVO_MAX  (450)
#define SERVO_FREQ (50)

const int JOINT_STATE_MAX[__NUM_JOINTS__] = {165,165,165,165};
const int JOINT_STATE_MIN[__NUM_JOINTS__] = {0,0,0,0};
const int JOINT_STATE_DEFAULT[__NUM_JOINTS__] = {90,75,70,85};

//==============================================================================
//  Private Data Members
//==============================================================================

static int _joints[__NUM_JOINTS__];
static Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver(&Wire,SERVO_DRIVER_ADDRESS);

//==============================================================================
//  Private Function Prototypes
//==============================================================================

static void _set_servo(int channel, int angle);

//==============================================================================
//  Public Function Implementation
//==============================================================================

void fd_init(void) {

  servoDriver.begin();
  servoDriver.setPWMFreq(SERVO_FREQ);

  delay(10);
}

int fd_reset_all_joint_states(void) {
  int retVal = 0;

  for (int i=0; i<__NUM_JOINTS__; i++ ) {
    retVal += fd_reset_joint_state((JointId_t)(i));
  }

  return retVal;
}

int fd_reset_joint_state(JointId_t jointId) {
  if (jointId < 0 || jointId >= __NUM_JOINTS__) {
    return -1;
  }

  _joints[jointId] = JOINT_STATE_DEFAULT[jointId];
  _set_servo(jointId,_joints[jointId]);

  return 0;
}

int fd_set_joint_state(JointId_t jointId, int value) {
  if (jointId < 0 || jointId >= __NUM_JOINTS__) {
    return -1;
  }

  if (JOINT_STATE_MAX[jointId] < value || JOINT_STATE_MIN[jointId] > value) {
    return -2;
  } else {
    _joints[jointId] = value;
    _set_servo(jointId,value);
    return 0;
  }
}

int fd_get_joint_state(JointId_t jointId) {
  if (jointId < 0 || jointId >= __NUM_JOINTS__) {
    return -1;
  }

  return _joints[jointId];
}

//==============================================================================
//  Private Function Implementation
//==============================================================================

static void _set_servo(int channel, int angle) {
  int pulselength = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  servoDriver.setPWM(channel, 0, pulselength);
}
