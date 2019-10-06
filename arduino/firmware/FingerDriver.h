/**
 * Supernumerary Robotic Finger
 * Version 1.1.5
 * @author Curt Henrichs
 * @date 4-19-19
 *
 * Robotic Finger Joint Driver
 */

#ifndef FINGER_DRIVER_H
#define FINGER_DRIVER_H

//==============================================================================
//  Libraries
//==============================================================================

//==============================================================================
//  Constants and Macro Definitions
//==============================================================================

typedef enum JointId {

  // Available joints
  JOINT_0,
  JOINT_1,
  JOINT_2,
  JOINT_3,

  // Private value, used to define range of joints
  __NUM_JOINTS__
} JointId_t;

//==============================================================================
//  Public Function Prototypes
//==============================================================================

void fd_init(void);
int fd_reset_all_joint_states(void);
int fd_reset_joint_state(JointId_t jointId);
int fd_set_joint_state(JointId_t jointId, int value);
int fd_get_joint_state(JointId_t jointId);

#endif
