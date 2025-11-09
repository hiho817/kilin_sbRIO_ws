#ifndef __FSM_H
#define __FSM_H

#include <math.h>
#include <unistd.h>

#include <Eigen/Dense>
#include <fstream>
#include <vector>

#include "Motor.pb.h"
#include "Power.pb.h"
#include "hip_motor.hpp"

enum class Module_ID { L_MODULE, R_MODULE };

enum class Scenario { ROBOT, SINGLE_MODULE };

class ModeFsm {
 public:
  /* pass motors vector by reference*/
  ModeFsm(std::vector<HipMotor>* motors, std::vector<bool>* pb_state_);
  ModeFsm() {}
  Mode workingMode_;
  Mode prev_workingMode_;

  Scenario scenario_;

  std::vector<HipMotor>* hip_motor_list_;
  std::vector<bool>* pb_state_;

  bool hall_calibrated;
  int hall_calibrate_status;
  int impedance_status;

  int measure_offset = 0;
  double dt_ = 0.001;      // second
  double cal_vel_ = 0.25;  // rad/s
  double cal_tol_ = 0.05;
  double cal_dir_[4];
  double cal_command[4];

  bool* NO_CAN_TIMEDOUT_ERROR_;
  bool* NO_SWITCH_TIMEDOUT_ERROR_;

  void runFsm(motor_msg::MotorStateStamped& motor_fb_msg,
              const motor_msg::MotorCmdStamped& motor_cmd_msg);
  bool switchMode(Mode next_mode);
  void publishMsg(motor_msg::MotorStateStamped& motor_fb_msg);
};

#endif
