#ifndef __HIP_MOTOR_H
#define __HIP_MOTOR_H

#include <math.h>
#include <yaml-cpp/yaml.h>

#include <eigen3/Eigen/Dense>
#include <iomanip>
#include <iostream>
#include <string>

#include "fpga_handler.hpp"
#include "msg.hpp"

class HipMotor {
 public:
  HipMotor(std::string _label, YAML::Node _config, NiFpga_Status _status,
           NiFpga_Session _fpga_session, int _motor_index);
  HipMotor() {}

  std::string label_;
  YAML::Node config_;
  Motor_CAN motor_info_;
  int motor_index_;  // 0 or 1, indicates which motor on the CAN port

  ModuleIO_CAN io_;
  std::string CAN_port_;
  bool enable_;
  int CAN_timeout_us;
  bool CAN_first_transmit_;

  bool CAN_tx_timedout_;
  bool CAN_rx_timedout_;
  bool CAN_mtr_timedout_;

  CAN_txdata txdata_buffer_;
  CAN_rxdata rxdata_buffer_;

  // Current mode
  Mode current_mode_;
  Mode prev_mode_;

  void load_config();
  void CAN_timeoutCheck();
  
  // CAN communication methods
  void CAN_setup();
  void CAN_send_command();
  void CAN_receive_feedback();
  void update_motor();  // Main update function
  
  // Mode control methods
  bool switch_mode(Mode next_mode);
  bool is_mode(Mode mode) const { return current_mode_ == mode; }
  Mode get_mode() const { return current_mode_; }
  
  // Motor command methods
  void set_position_command(double pos, double kp, double ki, double kd);
  void set_torque_command(double torque);
  void set_control_gains(double kp, double ki, double kd);
  void stop();  // Set all commands to zero
  
  // Getters for feedback
  double get_position() const { return rxdata_buffer_.position_; }
  double get_velocity() const { return rxdata_buffer_.velocity_; }
  double get_torque() const { return rxdata_buffer_.torque_; }
  int get_calibrate_status() const { return rxdata_buffer_.calibrate_finish_; }
  
 private:
  // CAN packet encoding/decoding
  void CAN_encode_(uint8_t (&txmsg)[8], const CAN_txdata& txdata);
  void CAN_decode_(const uint8_t (&rxmsg)[8], CAN_rxdata* rxdata);
  
  // Data conversion for CAN-bus
  int float_to_uint_(float x, float x_min, float x_max, int bits);
  float uint_to_float_(int x_int, float x_min, float x_max, int bits);
  
  // Mode switching helper
  void set_mode_internal_(Mode mode);
};

#endif
