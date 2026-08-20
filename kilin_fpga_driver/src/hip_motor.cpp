#include <hip_motor.hpp>

HipMotor::HipMotor(std::string label, YAML::Node config, NiFpga_Status _status,
                   NiFpga_Session _fpga_session, int _motor_index, ModuleIO_CAN* shared_io)
    : label_(label), config_(config), motor_index_(_motor_index), enable_(false), io_(shared_io) {
  load_config();

  CAN_tx_timedout_ = false;
  CAN_rx_timedout_ = false;
  CAN_mtr_timedout_ = false;
  
  // Initialize mode
  current_mode_ = Mode::REST;
  prev_mode_ = Mode::REST;

  txdata_buffer_.position_ = 0;
  txdata_buffer_.torque_ = 0;
  txdata_buffer_.KP_ = motor_info_.kp_;
  txdata_buffer_.KI_ = motor_info_.ki_;
  txdata_buffer_.KD_ = motor_info_.kd_;
  txdata_buffer_.KT_ = motor_info_.kt_;

  rxdata_buffer_.mode_ = Mode::REST;
  rxdata_buffer_.mode_state_ = _REST_MODE;
  rxdata_buffer_.position_ = 0;
  rxdata_buffer_.torque_ = 0;
  rxdata_buffer_.velocity_ = 0;
  rxdata_buffer_.angle_difference_deg_ = 0;
  rxdata_buffer_.CAN_id_ = 0;

  // io_ is already set from constructor parameter (shared between motors)

  /* setup motor CAN ID, port selection and timeout_us */
  CAN_setup();
  
  /* Set initial FC to match current_mode_ (REST) */
  set_mode_internal_(current_mode_);
}

void HipMotor::load_config() {
  CAN_timeout_us = config_["CAN_Timeout_us"].as<int>();

  // load configuration from yaml file
  std::cout << "[ " << label_ << " Motor Configuration ]" << std::endl;
  enable_ = config_[label_]["Enable"].as<int>();
  CAN_port_ = config_[label_]["CAN_PORT"].as<std::string>();

  // Determine which motor (F or H) based on motor_index_
  std::string motor_key = (motor_index_ == 0) ? "Motor_F" : "Motor_H";
  
  motor_info_.fw_version_ = config_[label_][motor_key]["FW_Version"].as<int>();
  motor_info_.CAN_ID_ = config_[label_][motor_key]["CAN_ID"].as<int>();
  motor_info_.kp_ = config_[label_][motor_key]["KP"].as<double>();
  motor_info_.ki_ = config_[label_][motor_key]["KI"].as<double>();
  motor_info_.kd_ = config_[label_][motor_key]["KD"].as<double>();
  motor_info_.kt_ = config_[label_][motor_key]["KT"].as<double>();
  motor_info_.torque_ff_ = config_[label_][motor_key]["Torque_Feedfoward"].as<double>();
  motor_info_.calibration_bias = 0;

  std::cout << "CAN PORT: " << CAN_port_ << " Motor Index: " << motor_index_ << std::endl;
  std::cout << std::setw(14) << "  FW_Version: " << std::setw(13) << motor_info_.fw_version_
            << std::endl;
  std::cout << std::setw(14) << "  CAN_ID: " << std::setw(13) << motor_info_.CAN_ID_ << std::endl;
  std::cout << std::setw(14) << "  KP: " << std::setw(13) << motor_info_.kp_ << std::endl;
  std::cout << std::setw(14) << "  KI: " << std::setw(13) << motor_info_.ki_ << std::endl;
  std::cout << std::setw(14) << "  KD: " << std::setw(13) << motor_info_.kd_ << std::endl;
  std::cout << std::setw(14) << "  KT: " << std::setw(13) << motor_info_.kt_ << std::endl;
  std::cout << std::setw(14) << "  Torque_ff: " << std::setw(13) << motor_info_.torque_ff_ << std::endl;
  std::cout << std::setw(14) << "---------------------------" << std::endl;
}

void HipMotor::CAN_timeoutCheck() {
  if (!io_) {
    return;  // Safety check - io_ pointer must be valid
  }
  
  CAN_rx_timedout_ = io_->get_ni_rx_timeout();
  CAN_tx_timedout_ = io_->get_ni_tx_timeout();
  CAN_mtr_timedout_ = CAN_rx_timedout_ || CAN_tx_timedout_;
}

void HipMotor::CAN_setup() {
  // Set this motor's CAN ID individually (won't overwrite the other motor's ID)
  if (motor_index_ == 0) {
    std::cout << "  Setting CAN ID1 = " << motor_info_.CAN_ID_ << " for " << label_ << " Motor_F" << std::endl;
    io_->set_ni_CAN_id1(motor_info_.CAN_ID_);
  } else {
    std::cout << "  Setting CAN ID2 = " << motor_info_.CAN_ID_ << " for " << label_ << " Motor_H" << std::endl;
    io_->set_ni_CAN_id2(motor_info_.CAN_ID_);
  }

  /* Enable BOTH motors on the CAN bus (both ID1 and ID2) */
  NiFpga_Bool _bool_arr[2] = {1, 1};  // Enable both motors
  io_->set_ni_port_select(_bool_arr);

  io_->set_ni_timeout_us(CAN_timeout_us);
}

void HipMotor::CAN_send_command() {
  if (!io_) {
    return;  // Safety check - io_ pointer must be valid
  }
  
  // If in HALL_CALIBRATE mode, send all zeros
  CAN_txdata tx_data_to_send;
  if (current_mode_ == Mode::HALL_CALIBRATE) {
    tx_data_to_send.position_ = 0;
    tx_data_to_send.torque_ = 0;
    tx_data_to_send.KP_ = 0;
    tx_data_to_send.KI_ = 0;
    tx_data_to_send.KD_ = 0;
    tx_data_to_send.KT_ = 0;
  } else {
    tx_data_to_send = txdata_buffer_;
  }
  
  uint8_t txmsg[8];
  CAN_encode_(txmsg, tx_data_to_send);

  uint32_t fc1, fc2;
  io_->get_ni_CAN_id_fc(&fc1, &fc2);

  if (motor_index_ == 0 && fc1 == 1) {
    txmsg[0] = 255;
  } else if (motor_index_ == 1 && fc2 == 1) {
    txmsg[0] = 255;
  }

  // Each motor sets its own TX data in the shared IO buffer
  if (motor_index_ == 0) {
    io_->set_ni_tx_data_motor1(txmsg);
  } else {
    io_->set_ni_tx_data_motor2(txmsg);
  }
}

void HipMotor::CAN_receive_feedback() {
  if (!io_) {
    return;  // Safety check - io_ pointer must be valid
  }
  
  uint8_t rxmsg_id1[8];
  uint8_t rxmsg_id2[8];
  io_->get_ni_rx_data(rxmsg_id1, rxmsg_id2);
  
  if (motor_index_ == 0) {
    CAN_decode_(rxmsg_id1, &rxdata_buffer_);
  } else {
    CAN_decode_(rxmsg_id2, &rxdata_buffer_);
  }
  
  // Don't auto-switch here to avoid recursion
  // Mode transitions are controlled by the server; feedback includes the
  // current FSM state and the MU150-to-motor angle difference.
}

void HipMotor::CAN_encode_(uint8_t (&txmsg)[8], const CAN_txdata& txdata) {
  int pos_int, torque_int, KP_int, KI_int, KD_int;
  pos_int = float_to_uint_(-txdata.position_, P_CMD_MIN, P_CMD_MAX, 16);
  KP_int = float_to_uint_(txdata.KP_, KP_MIN, KP_MAX, 12);
  KI_int = float_to_uint_(txdata.KI_, KI_MIN, KI_MAX, 12);
  KD_int = float_to_uint_(txdata.KD_, KD_MIN, KD_MAX, 12);
  torque_int = float_to_uint_(txdata.torque_, T_MIN, T_MAX, 12);

  txmsg[0] = pos_int >> 8;
  txmsg[1] = pos_int & 0xFF;
  txmsg[2] = KP_int >> 4;
  txmsg[3] = ((KP_int & 0x0F) << 4) | (KI_int >> 8);
  txmsg[4] = KI_int & 0xFF;
  txmsg[5] = KD_int >> 4;
  txmsg[6] = ((KD_int & 0x0F) << 4) | (torque_int >> 8);
  txmsg[7] = torque_int & 0xFF;
}

void HipMotor::CAN_decode_(const uint8_t (&rxmsg)[8], CAN_rxdata* rxdata) {
  int pos_raw, vel_raw, torque_raw, mode_raw;
  uint16_t angle_difference_raw;

  pos_raw = ((int)(rxmsg[0]) << 8) | rxmsg[1];
  vel_raw = ((int)(rxmsg[2]) << 8) | rxmsg[3];
  torque_raw = ((int)(rxmsg[4]) << 8) | rxmsg[5];
  angle_difference_raw = (static_cast<uint16_t>(rxmsg[6]) << 4) |
                         ((static_cast<uint16_t>(rxmsg[7]) >> 4) & 0x0F);
  mode_raw = ((int)(rxmsg[7] & 0x0F));

  rxdata->position_ = -uint_to_float_(pos_raw, P_FB_MIN, P_FB_MAX, 16);
  rxdata->velocity_ = uint_to_float_(vel_raw, V_MIN, V_MAX, 16);
  rxdata->torque_ = uint_to_float_(torque_raw, T_MIN, T_MAX, 16);
  rxdata->angle_difference_deg_ =
      uint_to_float_(angle_difference_raw, MU150_CAN_DIFF_MIN_DEG, MU150_CAN_DIFF_MAX_DEG, 12);
  rxdata->mode_state_ = mode_raw;

  if (mode_raw == _SET_ZERO)
    rxdata->mode_ = Mode::SET_ZERO;
  else if (mode_raw == _MOTOR_MODE)
    rxdata->mode_ = Mode::MOTOR;
  else if (mode_raw == _HALL_CALIBRATE)
    rxdata->mode_ = Mode::HALL_CALIBRATE;
  else if (mode_raw == _REST_MODE)
    rxdata->mode_ = Mode::REST;
}

int HipMotor::float_to_uint_(float x, float x_min, float x_max, int bits) {
  /// Converts a float to an unsigned int, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float HipMotor::uint_to_float_(int x_int, float x_min, float x_max, int bits) {
  /// converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

// Mode switching with retry logic
bool HipMotor::switch_mode(Mode next_mode) {
  if (!enable_) {
    return false;
  }

  bool success = false;
  double time_elapsed = 0;
  const double timeout = 0.02;  // 0.1 second timeout

  while (!success && time_elapsed < timeout) {
    // Set the mode
    set_mode_internal_(next_mode);
    
    // Send and receive
    CAN_send_command();
    io_->set_ni_CAN_transmit(true);
    usleep(1000);  // Short delay for transmission
    CAN_receive_feedback();

    // Check if mode switched
    if (next_mode == Mode::SET_ZERO) {
      // For SET_ZERO, check if position is near zero
      if (fabs(rxdata_buffer_.position_) <= 0.01) {
        success = true;
      }
    } else if (next_mode == Mode::CONFIG) {
      // For CONFIG, motor doesn't report CONFIG state back
      // Just accept it immediately (FC is set, motor accepts config writes)
      success = true;
    } else if (next_mode == Mode::HALL_CALIBRATE) {
      // For HALL_CALIBRATE, motor handles calibration autonomously
      // Just accept it immediately without waiting for feedback
      success = true;
    } else if (next_mode == Mode::MOTOR) {
      // For MOTOR mode, check mode feedback
      if (rxdata_buffer_.mode_ == next_mode) {
        success = true;
      }
    } else {
      // For other modes, check mode feedback
      if (rxdata_buffer_.mode_ == next_mode) {
        success = true;
      }
    }

    if (!success) {
      time_elapsed += 0.01;
      usleep(10000);  // 10ms
    }
  }

  if (success) {
    prev_mode_ = current_mode_;
    current_mode_ = next_mode;
    
    // After starting HALL_CALIBRATE, switch FC to CONFIG for the calibration flow.
    if (next_mode == Mode::HALL_CALIBRATE) {
      usleep(10000);  // Small delay after starting calibration
      set_mode_internal_(Mode::CONFIG);  // Switch to CONFIG FC to monitor calibration
    }
    
    // After entering MOTOR mode, immediately switch FC to CONTROL for position commands
    // This ensures FC goes from 4 to 5 automatically
    if (next_mode == Mode::MOTOR) {
      usleep(10000);  // Small delay to ensure MOTOR mode is confirmed
      set_mode_internal_(Mode::CONTROL);
    }
  }

  return success;
}

// Set mode in CAN function code
void HipMotor::set_mode_internal_(Mode mode) {
  uint32_t fc1, fc2;
  io_->get_ni_CAN_id_fc(&fc1, &fc2);
  
  if (motor_index_ == 0) {
    io_->set_ni_CAN_id_fc((int)mode, fc2);
  } else {
    io_->set_ni_CAN_id_fc(fc1, (int)mode);
  }
}

// Main update function - call this regularly
void HipMotor::update_motor() {
  if (!enable_) {
    return;
  }
  
  CAN_send_command();
  CAN_receive_feedback();
  CAN_timeoutCheck();
}

// Set position command with gains
void HipMotor::set_position_command(double pos, double kp, double ki, double kd) {
  txdata_buffer_.position_ = pos;
  txdata_buffer_.KP_ = kp;
  txdata_buffer_.KI_ = ki;
  txdata_buffer_.KD_ = kd;
}

// Set torque command
void HipMotor::set_torque_command(double torque) {
  txdata_buffer_.torque_ = torque;
}

// Set control gains
void HipMotor::set_control_gains(double kp, double ki, double kd) {
  txdata_buffer_.KP_ = kp;
  txdata_buffer_.KI_ = ki;
  txdata_buffer_.KD_ = kd;
}

// Stop motor - zero all commands
void HipMotor::stop() {
  txdata_buffer_.position_ = 0;
  txdata_buffer_.torque_ = 0;
  txdata_buffer_.KP_ = 0;
  txdata_buffer_.KI_ = 0;
  txdata_buffer_.KD_ = 0;
}
