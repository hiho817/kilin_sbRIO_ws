#include <cstring>
#include <limb_module.hpp>

LimbModule::LimbModule(std::string _label, YAML::Node _config, NiFpga_Status _status, NiFpga_Session _fpga_session,
                       int rs485_port)
    : label_(_label),
      status_(_status),
      fpga_session_(_fpga_session),
      rs485_port_(rs485_port),
      io_(_status, _fpga_session, rs485_port) {
  // Initialize motors
  steering_motor.id_ = 1;
  steering_motor.is_steering_ = true;
  steering_motor.pos_des_.as_float = 0.0f;
  steering_motor.vel_des_ = 0.0;
  steering_motor.trq_des_ = 0.0;
  steering_motor.pos_act_.as_float = 0.0f;
  steering_motor.vel_act_ = 0.0;
  steering_motor.trq_act_ = 0.0;
  steering_motor.mode_des_ = MotorMode::REST;
  steering_motor.mode_act_ = MotorMode::REST;

  wheel_motor.id_ = 2;
  wheel_motor.is_steering_ = false;
  wheel_motor.pos_des_.as_int = 0;
  wheel_motor.vel_des_ = 0.0;
  wheel_motor.trq_des_ = 0.0;
  wheel_motor.pos_act_.as_int = 0;
  wheel_motor.vel_act_ = 0.0;
  wheel_motor.trq_act_ = 0.0;
  wheel_motor.mode_des_ = MotorMode::REST;
  wheel_motor.mode_act_ = MotorMode::REST;

  // Initialize timeout flags
  RS485_tx_timedout = false;
  RS485_rx_timedout = false;
  RS485_module_timedout = false;

  // Initialize counters for timeout detection
  last_tx_count_ = 0;
  last_rx_count_ = 0;

  // Initialize mode change flags
  prev_mode_des_steering_ = MotorMode::REST;
  prev_mode_des_wheel_ = MotorMode::REST;
  mode_change_sent_steering_ = false;
  mode_change_sent_wheel_ = false;

  // Initialize calibration state flags
  cal_sent_steering_ = false;

  // Initialize TX buffer (no header/checksum, FPGA driver handles those)
  memset(&txdata_buffer_, 0, sizeof(txdata_buffer_));

  // Initialize RX buffer
  memset(&rxdata_buffer_, 0, sizeof(rxdata_buffer_));

  load_config();

  std::cout << "[" << label_ << "] Initialized on RS485 port " << rs485_port_ << std::endl;
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

bool LimbModule::pack_tx_buffer() {
  // Check if mode change is needed for each motor independently
  // A mode change is needed if:
  // 1. Desired mode is a motor command mode (POSITION/VELOCITY/TORQUE)
  // 2. AND desired mode changed since last cycle (prev_mode_des_ != mode_des_)
  // 3. AND no mode change command is currently pending (waiting for acknowledgment)

  bool need_mode_change_steering = false;
  bool need_mode_change_wheel = false;

  // For steering motor (only supports POSITION)
  if (steering_motor.mode_des_ == MotorMode::POSITION && prev_mode_des_steering_ != steering_motor.mode_des_ &&
      !mode_change_sent_steering_) {
    need_mode_change_steering = true;
  } else if (!RS485_module_timedout && mode_change_sent_steering_) {
    // Mode change command was acknowledged (no timeout), clear the flag
    mode_change_sent_steering_ = false;
  }

  // For wheel motor (supports POSITION/VELOCITY/TORQUE)
  if ((wheel_motor.mode_des_ == MotorMode::POSITION || wheel_motor.mode_des_ == MotorMode::VELOCITY ||
       wheel_motor.mode_des_ == MotorMode::TORQUE) &&
      prev_mode_des_wheel_ != wheel_motor.mode_des_ && !mode_change_sent_wheel_) {
    need_mode_change_wheel = true;
  } else if (!RS485_module_timedout && mode_change_sent_wheel_) {
    // Mode change command was acknowledged (no timeout), clear the flag
    mode_change_sent_wheel_ = false;
  }

  // Pack steering motor command
  if (need_mode_change_steering) {
    // Steering motor needs mode change
    txdata_buffer_.CMD1 = CMD_MOTOR_MODE;
    txdata_buffer_.SUBCMD1 = SUBCMD_MOTOR_POSITON;  // Steering only supports position
    txdata_buffer_.Data1 = 0;
    mode_change_sent_steering_ = true;  // Mark that mode change command is sent
  } else {
    // Steering motor sends normal command
    txdata_buffer_.Data1 = 0;
    switch (steering_motor.mode_des_) {
      case MotorMode::REST:
        txdata_buffer_.CMD1 = CMD_RESET;
        cal_sent_steering_ = false;  // Clear calibration flag when leaving calibration mode
        break;
      case MotorMode::SET_ZERO:
        txdata_buffer_.CMD1 = CMD_SET_ZERO;
        break;
      case MotorMode::HALL_CALIBRATE:
        if (!cal_sent_steering_) {
          // First time in calibration mode - send calibration command
          txdata_buffer_.CMD1 = CMD_HAL_CAL;
          cal_sent_steering_ = true;
        } else {
          // Calibration command already sent - keep sending echo mode
          txdata_buffer_.CMD1 = CMD_ECHO;
        }
        break;
      case MotorMode::POSITION:
        txdata_buffer_.CMD1 = CMD_MOTOR_CMD;
        // Reinterpret float as int32_t for transmission
        txdata_buffer_.Data1 = *reinterpret_cast<int32_t*>(&steering_motor.pos_des_.as_float);
        break;
      default:
        txdata_buffer_.CMD1 = CMD_RESET;
        break;
    }
  }

  // Pack wheel motor command
  if (need_mode_change_wheel) {
    // Wheel motor needs mode change
    txdata_buffer_.CMD2 = CMD_MOTOR_MODE;
    txdata_buffer_.Data2 = 0;

    switch (wheel_motor.mode_des_) {
      case MotorMode::VELOCITY:
        txdata_buffer_.SUBCMD2 = SUBCMD_MOTOR_SPEED;
        break;
      case MotorMode::TORQUE:
        txdata_buffer_.SUBCMD2 = SUBCMD_MOTOR_TORQUE;
        break;
      default:  // POSITION
        txdata_buffer_.SUBCMD2 = SUBCMD_MOTOR_POSITON;
        break;
    }
    mode_change_sent_wheel_ = true;  // Mark that mode change command is sent
  } else {
    // Wheel motor sends normal command
    switch (wheel_motor.mode_des_) {
      case MotorMode::REST:
        txdata_buffer_.CMD2 = CMD_RESET;
        txdata_buffer_.Data2 = 0;
        break;
      case MotorMode::SET_ZERO:
        txdata_buffer_.CMD2 = CMD_SET_ZERO;
        txdata_buffer_.Data2 = 0;
        break;
      case MotorMode::HALL_CALIBRATE:
        txdata_buffer_.CMD2 = CMD_HAL_CAL;
        txdata_buffer_.Data2 = 0;
        break;
      case MotorMode::POSITION:
        txdata_buffer_.CMD2 = CMD_MOTOR_CMD;
        // Use int32_t directly for wheel motor position
        txdata_buffer_.Data2 = wheel_motor.pos_des_.as_int;
        break;
      case MotorMode::VELOCITY:
        txdata_buffer_.CMD2 = CMD_MOTOR_CMD;
        txdata_buffer_.Data2 = static_cast<int32_t>(wheel_motor.vel_des_);
        break;
      case MotorMode::TORQUE:
        txdata_buffer_.CMD2 = CMD_MOTOR_CMD;
        txdata_buffer_.Data2 = static_cast<int32_t>(wheel_motor.trq_des_);
        break;
      default:
        txdata_buffer_.CMD2 = CMD_RESET;
        txdata_buffer_.Data2 = 0;
        break;
    }
  }

  // Update previous mode to current mode for next cycle comparison
  // Only update if not currently waiting for mode change acknowledgment
  if (!mode_change_sent_steering_) {
    prev_mode_des_steering_ = steering_motor.mode_des_;
  }
  if (!mode_change_sent_wheel_) {
    prev_mode_des_wheel_ = wheel_motor.mode_des_;
  }

  // Return false if any motor is changing mode
  return !(need_mode_change_steering || need_mode_change_wheel);
}

// ============================================================================
// RX Buffer Unpacking
// ============================================================================

void LimbModule::unpack_rx_buffer() {
  // FPGA driver verifies header and checksum before passing data to application

  // Unpack steering motor data (Motor 1)
  // CMD1 contains firmware version (bits 4-6) and command echo (bits 0-3)
  uint8_t cmd1 = rxdata_buffer_.CMD1 & 0x0F;

  // Decode mode from CMD1
  switch (cmd1) {
    case CMD_RESET:
      steering_motor.mode_act_ = MotorMode::REST;
      break;
    case CMD_CONFIG:
      steering_motor.mode_act_ = MotorMode::CONFIG;
      break;
    case CMD_SET_ZERO:
      steering_motor.mode_act_ = MotorMode::SET_ZERO;
      break;
    case CMD_HAL_CAL:
      steering_motor.mode_act_ = MotorMode::HALL_CALIBRATE;
      break;
    case CMD_ECHO:
      // Echo mode means motor is still in calibration, keep HALL_CALIBRATE state
      steering_motor.mode_act_ = MotorMode::HALL_CALIBRATE;
      break;
    case CMD_MOTOR_CMD:
      // For CMD_MOTOR_CMD, SUBCMD is not in RX buffer
      // Assume POSITION mode for steering since steering only supports position
      steering_motor.mode_act_ = MotorMode::POSITION;
      break;
    default:
      steering_motor.mode_act_ = MotorMode::REST;
      break;
  }

  // POS1 contains position feedback as float32
  steering_motor.pos_act_.as_float = *reinterpret_cast<float*>(&rxdata_buffer_.POS1);

  // STAT1 contains hall status (bits 4-5) and system state (bits 0-3)
  // uint8_t steering_status = rxdata_buffer_.STAT1;

  // I1 contains current feedback (scaled by 100)
  // steering_motor.trq_act_ = static_cast<double>(rxdata_buffer_.I1) / 100.0;

  // Unpack wheel motor data (Motor 2)
  // CMD2 contains firmware version (bits 4-6) and command echo (bits 0-3)
  uint8_t cmd2 = rxdata_buffer_.CMD2 & 0x0F;

  // Decode mode from CMD2
  switch (cmd2) {
    case CMD_RESET:
      wheel_motor.mode_act_ = MotorMode::REST;
      break;
    case CMD_CONFIG:
      wheel_motor.mode_act_ = MotorMode::CONFIG;
      break;
    case CMD_SET_ZERO:
      wheel_motor.mode_act_ = MotorMode::SET_ZERO;
      break;
    case CMD_HAL_CAL:
      wheel_motor.mode_act_ = MotorMode::HALL_CALIBRATE;
      break;
    case CMD_MOTOR_CMD:
      // For CMD_MOTOR_CMD, we cannot determine exact mode (POSITION/VELOCITY/TORQUE)
      // from RX buffer alone since SUBCMD is not echoed back
      // Use the desired mode as a fallback
      wheel_motor.mode_act_ = wheel_motor.mode_des_;
      break;
    default:
      wheel_motor.mode_act_ = MotorMode::REST;
      break;
  }

  // POS2 contains position feedback as int32_t for wheel motor
  wheel_motor.pos_act_.as_int = rxdata_buffer_.POS2;

  // STAT2 contains hall status (bits 4-5) and system state (bits 0-3)
  // uint8_t wheel_status = rxdata_buffer_.STAT2;

  // I2 contains current feedback (scaled by 100)
  // wheel_motor.trq_act_ = static_cast<double>(rxdata_buffer_.I2) / 100.0;

  // Clear timeout flags on successful communication
  RS485_module_timedout = false;
  RS485_rx_timedout = false;
}

// ============================================================================
// RS485 Communication
// ============================================================================

void LimbModule::send_motor_commands() {
  // Pack TX buffer (automatically determines if mode change is needed)
  pack_tx_buffer();

  // Send via RS485
  io_.set_ni_tx_data(txdata_buffer_.bytes);
  io_.set_ni_RS485_transmit(NiFpga_True);
}

void LimbModule::receive_motor_feedback() {
  // Check if RX is finished
  // if (!io_.get_ni_rx_finish()) {
  //   return;
  // }
  // note: rx_finish is only used internally by FPGA driver

  // Check if checksum is OK
  if (!io_.get_ni_checksum_ok()) {
    // Checksum error - data received but corrupted (not a timeout)
    return;
  }

  // Get raw RX buffer from FPGA (includes header and checksum)
  io_.get_ni_rx_buf(raw_rx_buffer_);

  // Get RX data using union's byte array (20 bytes - parsed data without header/checksum)
  io_.get_ni_rx_data(rxdata_buffer_.bytes);

  // Unpack the buffer
  unpack_rx_buffer();
}

void LimbModule::update_motors() {
  // Send motor commands (pack_tx_buffer handles mode change logic internally)
  send_motor_commands();
  receive_motor_feedback();

  // Always check for timeouts
  RS485_timeoutCheck();
}

// ============================================================================
// Motor Control Methods
// ============================================================================

void LimbModule::set_steering_position(double position) {
  if (position > MAX_STEERING_POSITION) {
    position = MAX_STEERING_POSITION;
  } else if (position < -MAX_STEERING_POSITION) {
    position = -MAX_STEERING_POSITION;
  }
  
  steering_motor.mode_des_ = MotorMode::POSITION;
  steering_motor.pos_des_.as_float = static_cast<float>(position);
}

void LimbModule::set_steering_velocity(double velocity) {
  steering_motor.mode_des_ = MotorMode::VELOCITY;
  steering_motor.vel_des_ = velocity;
}

void LimbModule::set_steering_torque(double torque) {
  steering_motor.mode_des_ = MotorMode::TORQUE;
  steering_motor.trq_des_ = torque;
}

void LimbModule::set_wheel_velocity(double velocity) {
  wheel_motor.mode_des_ = MotorMode::VELOCITY;
  wheel_motor.vel_des_ = velocity;
}

void LimbModule::set_wheel_torque(double torque) {
  wheel_motor.mode_des_ = MotorMode::TORQUE;
  wheel_motor.trq_des_ = torque;
}

// ============================================================================
// Timeout Check
// ============================================================================

void LimbModule::RS485_timeoutCheck() {
  // Check TX/RX counts for timeout detection (per-instance counters)
  int32_t current_tx_count = io_.get_ni_tx_count();
  int32_t current_rx_count = io_.get_ni_rx_count();

  // Update timeout flags based on count changes
  // Both motors are in the same packet, so they share the same timeout status
  RS485_tx_timedout = (current_tx_count == last_tx_count_);
  RS485_rx_timedout = (current_rx_count == last_rx_count_);
  
  // Update module timeout flag
  RS485_module_timedout = RS485_tx_timedout || RS485_rx_timedout;

  last_tx_count_ = current_tx_count;
  last_rx_count_ = current_rx_count;
}
