#ifndef __LEGMODULE_H
#define __LEGMODULE_H

#include <iostream>
#include <vector>
#include <math.h>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>
#include <iomanip>

#include "msg.hpp"
#include "fpga_handler.hpp"

class LegModule
{
public:
  LegModule(std::string _label, YAML::Node _config, NiFpga_Status _status, NiFpga_Session _fpga_session);
  LegModule(){
  }

  // ID of Module (F,H)
  std::string label_;
  YAML::Node config_;
  std::vector<Motor> motors_list_;

  // hardware configuration
  ModuleIO io_;
  std::string CAN_port_;
  bool enable_;
  int CAN_timeout_us;
  bool CAN_first_transmit_;

  bool CAN_tx_timedout_[2];
  bool CAN_rx_timedout_[2];
  bool CAN_mtr_timedout[2];
  bool CAN_module_timedout;

  CAN_txdata txdata_buffer_[2];
  CAN_rxdata rxdata_buffer_[2];

  double Motor_F_bias = 0;
  double Motor_H_bias = 0;

  // ModuleIO
  void load_config();
  void CAN_timeoutCheck();
};

#endif