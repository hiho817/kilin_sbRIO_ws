#include <limb_module.hpp>
#include <cstring>

LimbModule::LimbModule(std::string _label, YAML::Node _config, NiFpga_Status _status,
                       NiFpga_Session _fpga_session, int rs485_port)
    : label_(_label),
      status_(_status),
      fpga_session_(_fpga_session),
      rs485_port_(rs485_port),
      io_(_status, _fpga_session, rs485_port) {
  
  // Initialize motors
  steering_motor.id_ = 1;
  steering_motor.position_ = 0.0;
  steering_motor.velocity_ = 0.0;
  steering_motor.torque_ = 0.0;
  steering_motor.kp_ = 0.0;
  steering_motor.ki_ = 0.0;
  steering_motor.kd_ = 0.0;
  steering_motor.kt_ = 0.0;
  steering_motor.mode_ = MotorMode::REST;
  
  wheel_motor.id_ = 2;
  wheel_motor.position_ = 0.0;
  wheel_motor.velocity_ = 0.0;
  wheel_motor.torque_ = 0.0;
  wheel_motor.kp_ = 0.0;
  wheel_motor.ki_ = 0.0;
  wheel_motor.kd_ = 0.0;
  wheel_motor.kt_ = 0.0;
  wheel_motor.mode_ = MotorMode::REST;
  
  // Initialize timeout flags
  RS485_tx_timedout_[0] = false;
  RS485_tx_timedout_[1] = false;
  RS485_rx_timedout_[0] = false;
  RS485_rx_timedout_[1] = false;
  RS485_mtr_timedout[0] = false;
  RS485_mtr_timedout[1] = false;
  RS485_module_timedout = false;
  
  // Initialize TX buffer with header
  memset(&txdata_buffer_, 0, sizeof(txdata_buffer_));
  txdata_buffer_.Header = 0xFF;  // Will send 5 consecutive 0xFF
  
  // Initialize RX buffer
  memset(&rxdata_buffer_, 0, sizeof(rxdata_buffer_));
  
  load_config();
}

void LimbModule::load_config() {
  // RS485_timeout_us_ = config_["RS485_Timeout_us"].as<int>();
  RS485_timeout_us_ = 10000;  // Default 10ms timeout
}

// ============================================================================
// Checksum Calculation
// ============================================================================

uint8_t LimbModule::calculate_checksum(const uint8_t* data, size_t length) {
  uint8_t checksum = 0;
  // XOR all bytes except header and checksum bytes
  for (size_t i = 5; i < length - 2; i++) {
    checksum ^= data[i];
  }
  return checksum & 0xFE;
}

bool LimbModule::verify_checksum(const uint8_t* data, size_t length) {
  if (length < 3) return false;
  
  uint8_t calculated_cks1 = calculate_checksum(data, length);
  uint8_t calculated_cks2 = (~calculated_cks1) & 0xFE;
  
  uint8_t received_cks1 = data[length - 2];
  uint8_t received_cks2 = data[length - 1];
  
  return (calculated_cks1 == received_cks1) && (calculated_cks2 == received_cks2);
}

// ============================================================================
// TX Buffer Packing
// ============================================================================

void LimbModule::pack_tx_buffer() {
  // Pack the TX buffer according to protocol
  // Byte 0~4: Header (0xFF * 5)
  txdata_buffer_.Header = 0xFF;
  
  // Steering motor (Motor 1)
  txdata_buffer_.CMD1 = static_cast<uint8_t>(steering_motor.mode_);
  txdata_buffer_.SUBCMD1 = 0;  // Reserved for sub-commands
  
  // Data1 depends on mode
  switch (steering_motor.mode_) {
    case MotorMode::POSITION:
      txdata_buffer_.Data1 = static_cast<uint16_t>(steering_motor.position_ * 100);  // Scale as needed
      break;
    case MotorMode::VELOCITY:
      txdata_buffer_.Data1 = static_cast<uint16_t>(steering_motor.velocity_ * 100);
      break;
    case MotorMode::TORQUE:
      txdata_buffer_.Data1 = static_cast<uint16_t>(steering_motor.torque_ * 100);
      break;
    default:
      txdata_buffer_.Data1 = 0;
  }
  
  // Wheel motor (Motor 2)
  txdata_buffer_.CMD2 = static_cast<uint8_t>(wheel_motor.mode_);
  txdata_buffer_.SUBCMD2 = 0;
  
  // Data2 depends on mode
  switch (wheel_motor.mode_) {
    case MotorMode::POSITION:
      txdata_buffer_.Data2 = static_cast<uint16_t>(wheel_motor.position_ * 100);
      break;
    case MotorMode::VELOCITY:
      txdata_buffer_.Data2 = static_cast<uint16_t>(wheel_motor.velocity_ * 100);
      break;
    case MotorMode::TORQUE:
      txdata_buffer_.Data2 = static_cast<uint16_t>(wheel_motor.torque_ * 100);
      break;
    default:
      txdata_buffer_.Data2 = 0;
  }
  
  // Calculate checksums
  uint8_t* buffer_bytes = reinterpret_cast<uint8_t*>(&txdata_buffer_);
  txdata_buffer_.Checksum1 = calculate_checksum(buffer_bytes, sizeof(txdata_buffer_));
  txdata_buffer_.Checksum2 = (~txdata_buffer_.Checksum1) & 0xFE;
}

// ============================================================================
// RX Buffer Unpacking
// ============================================================================

void LimbModule::unpack_rx_buffer() {
  // Verify header
  if (rxdata_buffer_.Header != 0xFF) {
    RS485_module_timedout = true;
    return;
  }
  
  // Verify checksum
  uint8_t* buffer_bytes = reinterpret_cast<uint8_t*>(&rxdata_buffer_);
  if (!verify_checksum(buffer_bytes, sizeof(rxdata_buffer_))) {
    RS485_module_timedout = true;
    return;
  }
  
  // Unpack steering motor data (Motor 1)
  // CMD1 contains firmware version (bits 4-6) and command echo (bits 0-3)
  steering_motor.mode_ = static_cast<MotorMode>(rxdata_buffer_.CMD1 & 0x0F);
  steering_motor.position_ = static_cast<double>(rxdata_buffer_.POS1) / 100.0;  // Scale back
  // STAT1 contains hall-status (bits 4-5) and system state (bits 0-3)
  steering_motor.torque_ = static_cast<double>(rxdata_buffer_.I1) / 1000.0;  // Current to torque
  
  // Unpack wheel motor data (Motor 2)
  wheel_motor.mode_ = static_cast<MotorMode>(rxdata_buffer_.CMD2 & 0x0F);
  wheel_motor.position_ = static_cast<double>(rxdata_buffer_.POS2) / 100.0;
  wheel_motor.torque_ = static_cast<double>(rxdata_buffer_.I2) / 1000.0;
  
  // Clear timeout flags on successful communication
  RS485_module_timedout = false;
  RS485_rx_timedout_[0] = false;
  RS485_rx_timedout_[1] = false;
}

// ============================================================================
// RS485 Communication
// ============================================================================

void LimbModule::send_motor_commands() {
  // Pack the TX buffer
  pack_tx_buffer();
  
  // Send via RS485
  uint8_t* tx_bytes = reinterpret_cast<uint8_t*>(&txdata_buffer_);
  io_.set_ni_tx_data(tx_bytes, sizeof(txdata_buffer_));
  
  // Trigger transmission
  io_.set_ni_RS485_transmit(NiFpga_True);
}

void LimbModule::receive_motor_feedback() {
  // Check if RX is finished
  if (!io_.get_ni_rx_finish()) {
    return;
  }
  
  // Check if checksum is OK
  if (!io_.get_ni_checksum_ok()) {
    RS485_module_timedout = true;
    return;
  }
  
  // Get RX data
  uint8_t* rx_bytes = reinterpret_cast<uint8_t*>(&rxdata_buffer_);
  size_t length = sizeof(rxdata_buffer_);
  io_.get_ni_rx_data(rx_bytes, &length);
  
  // Unpack the buffer
  unpack_rx_buffer();
}

void LimbModule::update_motors() {
  send_motor_commands();
  receive_motor_feedback();
  RS485_timeoutCheck();
}

// ============================================================================
// Motor Control Methods
// ============================================================================

void LimbModule::set_steering_position(double position, double kp, double kd) {
  steering_motor.mode_ = MotorMode::POSITION;
  steering_motor.position_ = position;
  steering_motor.kp_ = kp;
  steering_motor.kd_ = kd;
}

void LimbModule::set_steering_velocity(double velocity, double kp, double kd) {
  steering_motor.mode_ = MotorMode::VELOCITY;
  steering_motor.velocity_ = velocity;
  steering_motor.kp_ = kp;
  steering_motor.kd_ = kd;
}

void LimbModule::set_steering_torque(double torque) {
  steering_motor.mode_ = MotorMode::TORQUE;
  steering_motor.torque_ = torque;
}

void LimbModule::set_wheel_velocity(double velocity, double kp, double kd) {
  wheel_motor.mode_ = MotorMode::VELOCITY;
  wheel_motor.velocity_ = velocity;
  wheel_motor.kp_ = kp;
  wheel_motor.kd_ = kd;
}

void LimbModule::set_wheel_torque(double torque) {
  wheel_motor.mode_ = MotorMode::TORQUE;
  wheel_motor.torque_ = torque;
}

// ============================================================================
// Timeout Check
// ============================================================================

void LimbModule::RS485_timeoutCheck() {
  // Check TX/RX counts for timeout detection
  static int32_t last_tx_count = 0;
  static int32_t last_rx_count = 0;
  
  int32_t current_tx_count = io_.get_ni_tx_count();
  int32_t current_rx_count = io_.get_ni_rx_count();
  
  // Update timeout flags based on count changes
  if (current_tx_count == last_tx_count) {
    RS485_tx_timedout_[0] = true;
    RS485_tx_timedout_[1] = true;
  } else {
    RS485_tx_timedout_[0] = false;
    RS485_tx_timedout_[1] = false;
  }
  
  if (current_rx_count == last_rx_count) {
    RS485_rx_timedout_[0] = true;
    RS485_rx_timedout_[1] = true;
  } else {
    RS485_rx_timedout_[0] = false;
    RS485_rx_timedout_[1] = false;
  }
  
  // Update motor timeout flags
  RS485_mtr_timedout[0] = RS485_tx_timedout_[0] || RS485_rx_timedout_[0];
  RS485_mtr_timedout[1] = RS485_tx_timedout_[1] || RS485_rx_timedout_[1];
  
  // Update module timeout flag
  RS485_module_timedout = RS485_mtr_timedout[0] || RS485_mtr_timedout[1];
  
  last_tx_count = current_tx_count;
  last_rx_count = current_rx_count;
}
