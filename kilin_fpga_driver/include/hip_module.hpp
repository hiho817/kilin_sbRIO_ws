#ifndef __LEGMODULE_H
#define __LEGMODULE_H

#include <math.h>
#include <yaml-cpp/yaml.h>

#include <eigen3/Eigen/Dense>
#include <iomanip>
#include <iostream>
#include <vector>

#include "fpga_handler.hpp"
#include "msg.hpp"

class HipModule {
 public:
  HipModule(std::string _label, YAML::Node _config, NiFpga_Status _status,
            NiFpga_Session _fpga_session);
  HipModule() {}

  std::string label_;
  YAML::Node config_;
  std::vector<Motor_CAN> motors_list_;

  ModuleIO_CAN io_;
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

  void load_config();
  void CAN_timeoutCheck();
  
  // CAN communication methods
  void CAN_setup();
  void CAN_set_mode(Mode mode);
  void CAN_send_command();
  void CAN_receive_feedback();
  
 private:
  // CAN packet encoding/decoding
  void CAN_encode_(uint8_t (&txmsg)[8], const CAN_txdata& txdata);
  void CAN_decode_(const uint8_t (&rxmsg)[8], CAN_rxdata* rxdata);
  
  // Data conversion for CAN-bus
  int float_to_uint_(float x, float x_min, float x_max, int bits);
  float uint_to_float_(int x_int, float x_min, float x_max, int bits);
};

double deg2rad(double deg);
Eigen::Vector2d tb2phi(const Eigen::Vector2d& tb);
Eigen::Vector2d phi2tb(const Eigen::Vector2d& phi);

#endif
