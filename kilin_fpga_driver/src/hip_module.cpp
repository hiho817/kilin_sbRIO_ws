#include <hip_module.hpp>

HipModule::HipModule(std::string label, YAML::Node config, NiFpga_Status _status,
                     NiFpga_Session _fpga_session)
    : label_(label), config_(config), enable_(false) {
  load_config();

  CAN_tx_timedout_[0] = false;
  CAN_tx_timedout_[1] = false;
  CAN_rx_timedout_[0] = false;
  CAN_rx_timedout_[1] = false;

  CAN_mtr_timedout[0] = false;
  CAN_mtr_timedout[1] = false;

  CAN_module_timedout = false;

  txdata_buffer_[0].position_ = 0;
  txdata_buffer_[0].torque_ = 0;
  txdata_buffer_[0].KP_ = motors_list_[0].kp_;
  txdata_buffer_[0].KI_ = motors_list_[0].ki_;
  txdata_buffer_[0].KD_ = motors_list_[0].kd_;
  txdata_buffer_[0].KT_ = motors_list_[0].kt_;

  txdata_buffer_[1].position_ = 0;
  txdata_buffer_[1].torque_ = 0;
  txdata_buffer_[1].KP_ = motors_list_[1].kp_;
  txdata_buffer_[1].KI_ = motors_list_[1].ki_;
  txdata_buffer_[1].KD_ = motors_list_[1].kd_;
  txdata_buffer_[1].KT_ = motors_list_[1].kt_;

  rxdata_buffer_[0].mode_ = Mode::REST;
  rxdata_buffer_[0].mode_state_ = _REST_MODE;
  rxdata_buffer_[0].position_ = 0;
  rxdata_buffer_[0].torque_ = 0;
  rxdata_buffer_[0].velocity_ = 0;
  rxdata_buffer_[0].calibrate_finish_ = 0;
  rxdata_buffer_[0].CAN_id_ = 0;
  rxdata_buffer_[0].version_ = 0;

  rxdata_buffer_[1].mode_ = Mode::REST;
  rxdata_buffer_[1].mode_state_ = _REST_MODE;
  rxdata_buffer_[1].position_ = 0;
  rxdata_buffer_[1].torque_ = 0;
  rxdata_buffer_[1].velocity_ = 0;
  rxdata_buffer_[1].calibrate_finish_ = 0;
  rxdata_buffer_[1].CAN_id_ = 0;
  rxdata_buffer_[1].version_ = 0;

  io_ = ModuleIO_CAN(_status, _fpga_session, CAN_port_);
  CAN_first_transmit_ = true;

  /* setup motors' CAN ID, port selection and timeout_us */
  CAN_setup();
}

void HipModule::load_config() {
  Motor_CAN motor_f;
  Motor_CAN motor_h;
  CAN_timeout_us = config_["CAN_Timeout_us"].as<int>();

  // load configuration from yaml file
  std::cout << "[ " << label_ << " Configuration ]" << std::endl;
  enable_ = config_[label_]["Enable"].as<int>();
  CAN_port_ = config_[label_]["CAN_PORT"].as<std::string>();

  // Motor F setup
  motor_f.fw_version_ = config_[label_]["Motor_F"]["FW_Version"].as<int>();
  motor_f.CAN_ID_ = config_[label_]["Motor_F"]["CAN_ID"].as<int>();
  motor_f.kp_ = config_[label_]["Motor_F"]["KP"].as<double>();
  motor_f.ki_ = config_[label_]["Motor_F"]["KI"].as<double>();
  motor_f.kd_ = config_[label_]["Motor_F"]["KD"].as<double>();
  motor_f.kt_ = config_[label_]["Motor_F"]["KT"].as<double>();
  motor_f.torque_ff_ = config_[label_]["Motor_F"]["Torque_Feedfoward"].as<double>();

  Motor_F_bias = config_[label_]["Motor_F"]["Calibration_Bias"].as<double>();
  motor_f.calibration_bias = 0;

  // Motor H setup
  motor_h.fw_version_ = config_[label_]["Motor_H"]["FW_Version"].as<int>();
  motor_h.CAN_ID_ = config_[label_]["Motor_H"]["CAN_ID"].as<int>();
  motor_h.kp_ = config_[label_]["Motor_H"]["KP"].as<double>();
  motor_h.ki_ = config_[label_]["Motor_H"]["KI"].as<double>();
  motor_h.kd_ = config_[label_]["Motor_H"]["KD"].as<double>();
  motor_h.kt_ = config_[label_]["Motor_H"]["KT"].as<double>();
  motor_h.torque_ff_ = config_[label_]["Motor_H"]["Torque_Feedfoward"].as<double>();

  Motor_H_bias = config_[label_]["Motor_H"]["Calibration_Bias"].as<double>();
  motor_h.calibration_bias = 0;

  motors_list_.push_back(motor_f);
  motors_list_.push_back(motor_h);

  std::cout << "CAN PORT: " << config_[label_]["CAN_PORT"].as<std::string>() << std::endl;

  std::cout << "Motor_F: " << std::endl;
  std::cout << std::setw(14) << "  FW_Version: " << std::setw(13) << motor_f.fw_version_
            << std::endl;
  std::cout << std::setw(14) << "  CAN_ID: " << std::setw(13) << motor_f.CAN_ID_ << std::endl;
  std::cout << std::setw(14) << "  KP: " << std::setw(13) << motor_f.kp_ << std::endl;
  std::cout << std::setw(14) << "  KI: " << std::setw(13) << motor_f.ki_ << std::endl;
  std::cout << std::setw(14) << "  KD: " << std::setw(13) << motor_f.kd_ << std::endl;
  std::cout << std::setw(14) << "  KT: " << std::setw(13) << motor_f.kt_ << std::endl;
  std::cout << std::setw(14) << "  Torque_ff: " << std::setw(13) << motor_f.torque_ff_ << std::endl;
  std::cout << std::setw(14) << "  Bias: " << std::setw(13) << Motor_F_bias << std::endl;
  std::cout << std::setw(14) << "---------------------------" << std::endl;

  std::cout << "Motor_H: " << std::endl;
  std::cout << std::setw(14) << "  FW_Version: " << std::setw(13) << motor_h.fw_version_
            << std::endl;
  std::cout << std::setw(14) << "  CAN_ID: " << std::setw(13) << motor_h.CAN_ID_ << std::endl;
  std::cout << std::setw(14) << "  KP: " << std::setw(13) << motor_h.kp_ << std::endl;
  std::cout << std::setw(14) << "  KI: " << std::setw(13) << motor_h.ki_ << std::endl;
  std::cout << std::setw(14) << "  KD: " << std::setw(13) << motor_h.kd_ << std::endl;
  std::cout << std::setw(14) << "  KT: " << std::setw(13) << motor_h.kt_ << std::endl;
  std::cout << std::setw(14) << "  Torque_ff: " << std::setw(13) << motor_h.torque_ff_ << std::endl;
  std::cout << std::setw(14) << "  Bias: " << std::setw(13) << Motor_H_bias << std::endl;
  std::cout << std::setw(14) << "---------------------------" << std::endl;
}

void HipModule::CAN_timeoutCheck() {
  CAN_rx_timedout_[0] = io_.get_ni_rx_timeout();
  CAN_tx_timedout_[0] = io_.get_ni_tx_timeout();

  CAN_mtr_timedout[0] = CAN_rx_timedout_[0] || CAN_tx_timedout_[0];
  CAN_mtr_timedout[1] = CAN_rx_timedout_[1] || CAN_tx_timedout_[1];

  CAN_module_timedout = CAN_mtr_timedout[0] || CAN_mtr_timedout[1];
}

void HipModule::CAN_setup() {
  io_.set_ni_CAN_id(motors_list_[0].CAN_ID_, motors_list_[1].CAN_ID_);

  /* select two port to transceive */
  NiFpga_Bool _bool_arr[2] = {1, 1};
  io_.set_ni_port_select(_bool_arr);

  io_.set_ni_timeout_us(CAN_timeout_us);
}

void HipModule::CAN_set_mode(Mode mode) { 
  io_.set_ni_CAN_id_fc((int)mode, (int)mode); 
}

void HipModule::CAN_send_command() {
  uint8_t txmsg_id1[8];
  uint8_t txmsg_id2[8];
  CAN_txdata txdata1_biased;
  CAN_txdata txdata2_biased;

  txdata1_biased.position_ = txdata_buffer_[0].position_ + Motor_F_bias;
  txdata1_biased.torque_ = txdata_buffer_[0].torque_;
  txdata1_biased.KP_ = txdata_buffer_[0].KP_;
  txdata1_biased.KI_ = txdata_buffer_[0].KI_;
  txdata1_biased.KD_ = txdata_buffer_[0].KD_;

  txdata2_biased.position_ = txdata_buffer_[1].position_ + Motor_H_bias;
  txdata2_biased.torque_ = txdata_buffer_[1].torque_;
  txdata2_biased.KP_ = txdata_buffer_[1].KP_;
  txdata2_biased.KI_ = txdata_buffer_[1].KI_;
  txdata2_biased.KD_ = txdata_buffer_[1].KD_;

  CAN_encode_(txmsg_id1, txdata1_biased);
  CAN_encode_(txmsg_id2, txdata2_biased);

  uint32_t fc1, fc2;
  io_.get_ni_CAN_id_fc(&fc1, &fc2);

  if (fc1 == 1) txmsg_id1[0] = 255;
  if (fc2 == 1) txmsg_id2[0] = 255;

  io_.set_ni_tx_data(txmsg_id1, txmsg_id2);
  usleep(100);
  io_.set_ni_CAN_transmit(1);
}

void HipModule::CAN_receive_feedback() {
  uint8_t rxmsg_id1[8];
  uint8_t rxmsg_id2[8];
  io_.get_ni_rx_data(rxmsg_id1, rxmsg_id2);
  CAN_decode_(rxmsg_id1, &rxdata_buffer_[0]);
  CAN_decode_(rxmsg_id2, &rxdata_buffer_[1]);

  rxdata_buffer_[0].position_ -= Motor_F_bias;
  rxdata_buffer_[1].position_ -= Motor_H_bias;
}

void HipModule::CAN_encode_(uint8_t (&txmsg)[8], const CAN_txdata& txdata) {
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

void HipModule::CAN_decode_(const uint8_t (&rxmsg)[8], CAN_rxdata* rxdata) {
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

int HipModule::float_to_uint_(float x, float x_min, float x_max, int bits) {
  /// Converts a float to an unsigned int, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float HipModule::uint_to_float_(int x_int, float x_min, float x_max, int bits) {
  /// converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

double deg2rad(double deg) { return deg * M_PI / 180.0; }
