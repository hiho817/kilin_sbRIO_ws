#ifndef __MSG_H
#define __MSG_H

#include <math.h>

#include <string>
#include <vector>

#include "mode.hpp"

typedef struct Motor_CAN {
  int CAN_ID_;
  int fw_version_;
  double kp_;
  double ki_;
  double kd_;
  double torque_ff_;
  double calibration_bias;
  double kt_;
} Motor_CAN;

// transmitted to SBRIO
typedef struct CAN_txdata {
  float position_;
  float torque_;
  float KP_;
  float KI_;
  float KD_;
  float KT_;
} CAN_txdata;

typedef struct CAN_rxdata {
  int CAN_id_;
  float position_;
  float velocity_;
  float torque_;
  int version_;
  int cal_stat_;
  int mode_state_;
  Mode mode_;
} CAN_rxdata;


#endif