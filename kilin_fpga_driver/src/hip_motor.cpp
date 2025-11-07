#include <hip_motor.hpp>

HipMotor::HipMotor(std::string label, YAML::Node config, NiFpga_Status _status,
                   NiFpga_Session _fpga_session, int _motor_index)
    : label_(label), config_(config), motor_index_(_motor_index), enable_(false) {
  load_config();

  CAN_tx_timedout_ = false;
  CAN_rx_timedout_ = false;
  CAN_mtr_timedout_ = false;

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
  rxdata_buffer_.calibrate_finish_ = 0;
  rxdata_buffer_.CAN_id_ = 0;
  rxdata_buffer_.version_ = 0;

  io_ = ModuleIO_CAN(_status, _fpga_session, CAN_port_);
  CAN_first_transmit_ = true;

  /* setup motor CAN ID, port selection and timeout_us */
  CAN_setup();
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
  CAN_rx_timedout_ = io_.get_ni_rx_timeout();
  CAN_tx_timedout_ = io_.get_ni_tx_timeout();
  CAN_mtr_timedout_ = CAN_rx_timedout_ || CAN_tx_timedout_;
}

void HipMotor::CAN_setup() {
  // For a single motor, we still need to set both IDs but only use one
  // The other motor on the same CAN port will be handled by another HipMotor instance
  int other_motor_id = 0;  // Placeholder, will be overridden by the other motor instance
  
  if (motor_index_ == 0) {
    io_.set_ni_CAN_id(motor_info_.CAN_ID_, other_motor_id);
  } else {
    io_.set_ni_CAN_id(other_motor_id, motor_info_.CAN_ID_);
  }

  /* select the appropriate port */
  NiFpga_Bool _bool_arr[2] = {motor_index_ == 0 ? 1 : 0, motor_index_ == 1 ? 1 : 0};
  io_.set_ni_port_select(_bool_arr);

  io_.set_ni_timeout_us(CAN_timeout_us);
}

void HipMotor::CAN_set_mode(Mode mode) {
  // Set mode for this specific motor
  uint32_t fc1, fc2;
  io_.get_ni_CAN_id_fc(&fc1, &fc2);
  
  if (motor_index_ == 0) {
    io_.set_ni_CAN_id_fc((int)mode, fc2);
  } else {
    io_.set_ni_CAN_id_fc(fc1, (int)mode);
  }
}

void HipMotor::CAN_send_command() {
  uint8_t txmsg[8];
  CAN_encode_(txmsg, txdata_buffer_);

  uint32_t fc1, fc2;
  io_.get_ni_CAN_id_fc(&fc1, &fc2);

  if (motor_index_ == 0 && fc1 == 1) {
    txmsg[0] = 255;
  } else if (motor_index_ == 1 && fc2 == 1) {
    txmsg[0] = 255;
  }

  // Send to appropriate port
  uint8_t dummy[8] = {0};
  if (motor_index_ == 0) {
    io_.set_ni_tx_data(txmsg, dummy);
  } else {
    io_.set_ni_tx_data(dummy, txmsg);
  }
  
  usleep(100);
  io_.set_ni_CAN_transmit(1);
}

void HipMotor::CAN_receive_feedback() {
  uint8_t rxmsg_id1[8];
  uint8_t rxmsg_id2[8];
  io_.get_ni_rx_data(rxmsg_id1, rxmsg_id2);
  
  if (motor_index_ == 0) {
    CAN_decode_(rxmsg_id1, &rxdata_buffer_);
  } else {
    CAN_decode_(rxmsg_id2, &rxdata_buffer_);
  }
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
  int pos_raw, vel_raw, torque_raw, cal_raw, ver_raw, mode_raw;

  pos_raw = ((int)(rxmsg[0]) << 8) | rxmsg[1];
  vel_raw = ((int)(rxmsg[2]) << 8) | rxmsg[3];
  torque_raw = ((int)(rxmsg[4]) << 8) | rxmsg[5];
  cal_raw = ((int)(rxmsg[6] & 0x0F));
  ver_raw = ((int)(rxmsg[7] >> 4));
  mode_raw = ((int)(rxmsg[7] & 0x0F));

  rxdata->position_ = -uint_to_float_(pos_raw, P_FB_MIN, P_FB_MAX, 16);
  rxdata->velocity_ = uint_to_float_(vel_raw, V_MIN, V_MAX, 16);
  rxdata->torque_ = uint_to_float_(torque_raw, T_MIN, T_MAX, 16);
  rxdata->version_ = ver_raw;
  rxdata->calibrate_finish_ = cal_raw;
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
