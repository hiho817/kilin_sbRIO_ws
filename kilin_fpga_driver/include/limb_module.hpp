#ifndef __LIMBMODULE_H
#define __LIMBMODULE_H

#include <math.h>
#include <yaml-cpp/yaml.h>

#include <eigen3/Eigen/Dense>
#include <iomanip>
#include <iostream>
#include <vector>

#include "fpga_handler.hpp"
#include "msg.hpp"

class LimbModule {
 public:
  LimbModule(std::string _label, YAML::Node _config, NiFpga_Status _status, NiFpga_Session _fpga_session);
  LimbModule() {}

  // ID of Module (F,H)
  std::string label_;
  YAML::Node config_;
  std::vector<Motor> motors_list_;

  // hardware configuration
  /* RS485_IO io_; */
  std::string RS485_port_;
  bool enable_;
  int RS485_timeout_us;
  bool RS485_first_transmit_;

  void Helloworld() { std::cout << "Hello from LimbModule!" << std::endl; }

 private:
  bool RS485_tx_timedout_[2];
  bool RS485_rx_timedout_[2];
  bool RS485_mtr_timedout[2];
  bool RS485_module_timedout;

  /* RS485_txdata txdata_buffer_[2]; */
  /* RS485_rxdata rxdata_buffer_[2]; */

  double Motor_F_bias = 0;
  double Motor_H_bias = 0;

  void load_config();
  void RS485_timeoutCheck();
};

#endif
