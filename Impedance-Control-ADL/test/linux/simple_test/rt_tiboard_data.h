/*! @file ti_boardcontrol.h
 *  @brief TI Board Code, used to simulate the TI board
 *
 *  This is mostly a copy of the exact code that runs on the TI Board
 */

#ifndef TI_BOARDCONTROL_H
#define TI_BOARDCONTROL_H

#include "cTypes.h"

/*!
 * Command sent to TI board
 */
typedef struct{
  float position_des[3];
  float velocity_des[3];
  float kp[3];
  float kd[3];

  u32 enable; // uint32t
  u32 zero;   // uint32t

  float force_ff[3];
  float tau_ff[3];

  float zero_offset[3];

  float q_des[3];
  float qd_des[3];
  float kp_joint[3];
  float kd_joint[3];

  float max_torque; // todo this isn't used.

} TiBoardCommand;

/*!
 * Data received from TI board
 */
typedef struct {
  float position[3];
  float velocity[3];
  float force[3];
  float q[3];
  float dq[3];
  float tau[3];
  float tau_des[3]; // todo this is encoder ticks?
  u32 loop_count_ti;
  u32 ethercat_count_ti;
  u32 microtime_ti;
}TiBoardData;


#endif  // TI_BOARDCONTROL_H
