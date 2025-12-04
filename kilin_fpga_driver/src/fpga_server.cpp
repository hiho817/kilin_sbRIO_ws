#include "fpga_server.hpp"

/* TCP node connection setup*/
volatile int motor_message_updated = 0;
volatile int pwrb_message_updated = 0;  // power

std::mutex mutex_;

motor_msg::MotorCmdStamped motor_cmd_data;
void motor_data_cb(motor_msg::MotorCmdStamped motor_msg) {
  mutex_.lock();
  motor_message_updated = 1;
  motor_cmd_data = motor_msg;
  mutex_.unlock();
}

power_msg::PowerCmdStamped power_cmd_data;
void power_data_cb(power_msg::PowerCmdStamped power_msg) {
  mutex_.lock();
  pwrb_message_updated = 1;
  power_cmd_data = power_msg;
  mutex_.unlock();
}

volatile sig_atomic_t sys_stop = 0;
/* CAPTURE SYS STOP SIGNAL TO KILL PROCESS*/
void inthand(int signum) { sys_stop = 1; }

bool is_sys_stop() { return sys_stop; }

Kilin::Kilin() {
  /* default value of interrupt*/
  main_irq_period_us_ = 500;
  can_irq_period_us_ = 800;
  seq = 0;

  /* initialize powerboard state */
  digital_switch_ = false;
  signal_switch_ = false;
  power_switch_ = false;
  NO_CAN_TIMEDOUT_ERROR_ = true;
  NO_SWITCH_TIMEDOUT_ERROR_ = true;

  /* initialize robot state */
  HALL_CALIBRATED_ = false;
  max_timeout_cnt_ = 100;

  // Initialize CAN IO pointers to nullptr
  can_io_L_ = nullptr;
  can_io_R_ = nullptr;

  // Initialize motor pointers to nullptr
  hip_motor_LF_ = nullptr;
  hip_motor_LH_ = nullptr;
  hip_motor_RF_ = nullptr;
  hip_motor_RH_ = nullptr;

  // Initialize pending mode changes
  pending_mode_LF_.pending = false;
  pending_mode_LH_.pending = false;
  pending_mode_RF_.pending = false;
  pending_mode_RH_.pending = false;

  powerboard_state_.push_back(digital_switch_);
  powerboard_state_.push_back(signal_switch_);
  powerboard_state_.push_back(power_switch_);

  load_config_();
  console_.init_hip_motors(&fpga_, hip_motor_LF_, hip_motor_LH_, hip_motor_RF_, hip_motor_RH_,
                           &limb_modules_list_, &powerboard_state_, &main_mtx_);

  fpga_.set_ni_irq_period(main_irq_period_us_, can_irq_period_us_);

  // wait until enter key is pressed
  cout << "Press Enter to start FPGA server..." << endl;
  cin.get();
}

void Kilin::load_config_() {
  // Load YAML config file
  yaml_node_ = YAML::LoadFile(CONFIG_PATH);

  // load interrupt periods
  main_irq_period_us_ = yaml_node_["MainLoop_period_us"].as<int>();
  can_irq_period_us_ = yaml_node_["CANLoop_period_us"].as<int>();

  /* Initialize individual hip motors (4 motors total) */
  // Each motor is independent with its own mode control
  // But motors on the same CAN port share the same IO object
  cout << "Initializing individual hip motors..." << endl;

  // Read CAN port names from YAML configuration
  std::string can_port_L = yaml_node_["L_Module"]["CAN_PORT"].as<std::string>();
  std::string can_port_R = yaml_node_["R_Module"]["CAN_PORT"].as<std::string>();
  
  cout << "L_Module CAN port: " << can_port_L << endl;
  cout << "R_Module CAN port: " << can_port_R << endl;

  // Create shared CAN IO objects (one per CAN port)
  can_io_L_ = new ModuleIO_CAN(fpga_.get_fpga_status(), fpga_.session, can_port_L);
  can_io_R_ = new ModuleIO_CAN(fpga_.get_fpga_status(), fpga_.session, can_port_R);

  // Left motors (both share can_io_L_)
  hip_motor_LF_ =
      new HipMotor("L_Module", yaml_node_, fpga_.get_fpga_status(), fpga_.session, 0, can_io_L_);  // Motor_F
  hip_motor_LH_ =
      new HipMotor("L_Module", yaml_node_, fpga_.get_fpga_status(), fpga_.session, 1, can_io_L_);  // Motor_H

  // Right motors (both share can_io_R_)
  hip_motor_RF_ =
      new HipMotor("R_Module", yaml_node_, fpga_.get_fpga_status(), fpga_.session, 0, can_io_R_);  // Motor_F
  hip_motor_RH_ =
      new HipMotor("R_Module", yaml_node_, fpga_.get_fpga_status(), fpga_.session, 1, can_io_R_);  // Motor_H

  // Validation
  if (!can_io_L_ || !can_io_R_) {
    cout << "[ERROR] Failed to create CAN IO objects!" << endl;
  }
  if (!hip_motor_LF_ || !hip_motor_LH_ || !hip_motor_RF_ || !hip_motor_RH_) {
    cout << "[ERROR] Failed to create hip motor objects!" << endl;
  }

  cout << "Loaded 4 individual hip motors (LF, LH, RF, RH)" << std::endl;

  // Initialize limb modules for RS485 communication (4 modules, one per RS485 port)
  int rs485_modules_num_ = 4;
  for (int i = 0; i < rs485_modules_num_; i++) {
    // Each limb module uses a different RS485 port (1-4)
    LimbModule limb_module("LimbModule_" + std::to_string(i + 1), yaml_node_, fpga_.get_fpga_status(), fpga_.session,
                           i + 1);  // RS485 port number: 1, 2, 3, 4
    limb_modules_list_.push_back(limb_module);
    cout << "  Initialized " << limb_module.label_ << " on RS485 port " << (i + 1) << endl;
  }
  cout << "Added " << rs485_modules_num_ << " limb modules for RS485 communication." << std::endl;

  cout << "press Enter to continue..." << endl;
  cin.get();

  // initialize powerboard calibration parameters
  YAML::Node factors_node = yaml_node_["Powerboard_Scaling_Factor"];
  std::cout << "PowerBoard Scaling Factor" << std::endl;
  fpga_.pwrb_io.set_pwrb_cal_params_from_yml(factors_node);
  int idx_ = 0;
  for (auto f : factors_node) {
    std::cout << "Index " << idx_ << " Current Factor: " << fpga_.pwrb_io.get_i_factor(idx_)
              << ", Current Offset: " << fpga_.pwrb_io.get_i_offset(idx_) << std::endl
              << " Voltage Factor: " << fpga_.pwrb_io.get_v_factor(idx_)
              << ", Voltage Offset: " << fpga_.pwrb_io.get_v_offset(idx_) << std::endl;
    idx_++;
  }
}

void Kilin::interruptHandler(core::Subscriber<power_msg::PowerCmdStamped>& cmd_pb_sub_,
                             core::Publisher<power_msg::PowerStateStamped>& state_pb_pub_,
                             core::Subscriber<motor_msg::MotorCmdStamped>& cmd_sub_,
                             core::Publisher<motor_msg::MotorStateStamped>& state_pub_) {
  while (NiFpga_IsNotError(fpga_.get_fpga_status()) && !sys_stop) {
    uint32_t irqsAsserted;
    uint32_t irqTimeout = 10;  // ms
    NiFpga_Bool TimedOut = 0;

    // Wait on IRQ to ensure FPGA is ready
    fpga_.set_fpga_status(NiFpga_WaitOnIrqs(fpga_.session, fpga_.irqContext, NiFpga_Irq_0 | NiFpga_Irq_1, irqTimeout,
                                            &irqsAsserted, &TimedOut));

    if (NiFpga_IsError(fpga_.get_fpga_status())) {
      std::cout << red << "[FPGA Server] Error! Exiting program. LabVIEW error code: " << fpga_.get_fpga_status()
                << reset << std::endl;
    }

    uint32_t irq0_cnt;
    uint32_t irq1_cnt;

    if (TimedOut) {
      std::cout << red << "IRQ timedout" << ", IRQ_0 cnt: " << irq0_cnt << ", IRQ_1 cnt: " << irq1_cnt << reset
                << std::endl;
    }

    /* if an IRQ was asserted */
    if (NiFpga_IsNotError(fpga_.get_fpga_status()) && !TimedOut) {
      if (irqsAsserted & NiFpga_Irq_0) {
        mainLoop_(cmd_pb_sub_, state_pb_pub_, cmd_sub_, state_pub_);
        // Acknowledge IRQ to begin DMA acquisition
        fpga_.set_fpga_status(NiFpga_AcknowledgeIrqs(fpga_.session, irqsAsserted));
      }
      if (irqsAsserted & NiFpga_Irq_1) {
        canLoop_();
        rs485Loop_();  // Update RS485 limb modules

        // Acknowledge IRQ to begin DMA acquisition
        fpga_.set_fpga_status(NiFpga_AcknowledgeIrqs(fpga_.session, irqsAsserted));
      }
    }
    usleep(10);
  }
}

void Kilin::mainLoop_(core::Subscriber<power_msg::PowerCmdStamped>& cmd_pb_sub_,
                      core::Publisher<power_msg::PowerStateStamped>& state_pb_pub_,
                      core::Subscriber<motor_msg::MotorCmdStamped>& cmd_sub_,
                      core::Publisher<motor_msg::MotorStateStamped>& state_pub_) {
  fpga_.pwrb_io.set_ni_pwrb(&powerboard_state_);
  fpga_.pwrb_io.get_ni_pwrb_to_buf();

  core::spinOnce();
  mutex_.lock();
  power_msg::PowerStateStamped power_fb_msg;
  motor_msg::MotorStateStamped motor_fb_msg;

  // Process motor commands if new message received
  if (motor_message_updated == 1) {
    motorCommandUnpack(motor_cmd_data);
  }
  motor_message_updated = 0;

  mutex_.unlock();

  // Apply any pending mode changes (outside mutex - can block)
  applyPendingModeChanges();

  // Pack motor and power state messages
  powerboardPack(power_fb_msg);
  motorStatePack(motor_fb_msg);

  // Read Command
  mutex_.lock();
  if (power_cmd_data.clean() == true) {
    NO_CAN_TIMEDOUT_ERROR_ = true;
    NO_SWITCH_TIMEDOUT_ERROR_ = true;
    HALL_CALIBRATED_ = false;
    timeout_cnt_ = 0;
  }

  if (NO_SWITCH_TIMEDOUT_ERROR_) {
    if (pwrb_message_updated) {
      powerboard_state_.at(0) = power_cmd_data.digital();
      powerboard_state_.at(1) = power_cmd_data.signal();
      powerboard_state_.at(2) = power_cmd_data.power();

      // Note: robot_mode control was removed from PowerCmdStamped (grpc_core commit e49d0aa)
      // Mode control is now done per-motor via Motor.proto instead of at powerboard level
      
      pwrb_message_updated = 0;
    }
  }
  motor_fb_msg.mutable_header()->set_seq(seq);
  mutex_.unlock();
  state_pub_.publish(motor_fb_msg);
  state_pb_pub_.publish(power_fb_msg);
}

void Kilin::canLoop_() {
  // Update each motor individually
  // Power switch must be on (powerboard_state_.at(2))
  if (powerboard_state_.at(2) == true) {
    // Step 1: Send commands and transmit for left CAN bus
    if (hip_motor_LF_ && hip_motor_LF_->enable_) {
      hip_motor_LF_->CAN_send_command();
    }
    if (hip_motor_LH_ && hip_motor_LH_->enable_) {
      hip_motor_LH_->CAN_send_command();
    }
    if (can_io_L_) {
      usleep(100);
      can_io_L_->set_ni_CAN_transmit(1);
    }

    // Step 2: Send commands and transmit for right CAN bus
    if (hip_motor_RF_ && hip_motor_RF_->enable_) {
      hip_motor_RF_->CAN_send_command();
    }
    if (hip_motor_RH_ && hip_motor_RH_->enable_) {
      hip_motor_RH_->CAN_send_command();
    }
    if (can_io_R_) {
      usleep(100);
      can_io_R_->set_ni_CAN_transmit(1);
    }

    // Step 3: Receive feedback for all motors
    if (hip_motor_LF_ && hip_motor_LF_->enable_) {
      hip_motor_LF_->CAN_receive_feedback();
      hip_motor_LF_->CAN_timeoutCheck();
      if (hip_motor_LF_->CAN_mtr_timedout_)
        timeout_cnt_++;
      else
        timeout_cnt_ = 0;
    }

    if (hip_motor_LH_ && hip_motor_LH_->enable_) {
      hip_motor_LH_->CAN_receive_feedback();
      hip_motor_LH_->CAN_timeoutCheck();
      if (hip_motor_LH_->CAN_mtr_timedout_)
        timeout_cnt_++;
      else
        timeout_cnt_ = 0;
    }

    if (hip_motor_RF_ && hip_motor_RF_->enable_) {
      hip_motor_RF_->CAN_receive_feedback();
      hip_motor_RF_->CAN_timeoutCheck();
      if (hip_motor_RF_->CAN_mtr_timedout_)
        timeout_cnt_++;
      else
        timeout_cnt_ = 0;
    }

    if (hip_motor_RH_ && hip_motor_RH_->enable_) {
      hip_motor_RH_->CAN_receive_feedback();
      hip_motor_RH_->CAN_timeoutCheck();
      if (hip_motor_RH_->CAN_mtr_timedout_)
        timeout_cnt_++;
      else
        timeout_cnt_ = 0;
    }

    // Update timeout error flag
    if (timeout_cnt_ < max_timeout_cnt_) {
      NO_CAN_TIMEDOUT_ERROR_ = true;
    } else {
      NO_CAN_TIMEDOUT_ERROR_ = false;
    }
  }
}

void Kilin::rs485Loop_() {
  // Update all limb modules with RS485 communication
  for (size_t i = 0; i < limb_modules_list_.size(); i++) {
    if (powerboard_state_.at(2) == true) {  // Power switch is on
      limb_modules_list_[i].update_motors();
    }
  }
}

void Kilin::powerboardPack(power_msg::PowerStateStamped& power_dashboard_reply) {
  mutex_.lock();
  gettimeofday(&t_stamp, NULL);
  power_dashboard_reply.mutable_header()->set_seq(seq);
  power_dashboard_reply.mutable_header()->mutable_stamp()->set_sec(t_stamp.tv_sec);
  power_dashboard_reply.mutable_header()->mutable_stamp()->set_usec(t_stamp.tv_usec);

  power_dashboard_reply.set_digital(powerboard_state_.at(0));
  power_dashboard_reply.set_signal(powerboard_state_.at(1));
  power_dashboard_reply.set_power(powerboard_state_.at(2));

  if (NO_SWITCH_TIMEDOUT_ERROR_ == true && NO_CAN_TIMEDOUT_ERROR_ == true)
    power_dashboard_reply.set_clean(true);
  else
    power_dashboard_reply.set_clean(false);

  power_dashboard_reply.set_v_0(fpga_.pwrb_io.get_v_buf(0));
  power_dashboard_reply.set_i_0(fpga_.pwrb_io.get_i_buf(0));

  power_dashboard_reply.set_v_1(fpga_.pwrb_io.get_v_buf(1));
  power_dashboard_reply.set_i_1(fpga_.pwrb_io.get_i_buf(1));

  power_dashboard_reply.set_v_2(fpga_.pwrb_io.get_v_buf(2));
  power_dashboard_reply.set_i_2(fpga_.pwrb_io.get_i_buf(2));

  power_dashboard_reply.set_v_3(fpga_.pwrb_io.get_v_buf(3));
  power_dashboard_reply.set_i_3(fpga_.pwrb_io.get_i_buf(3));

  power_dashboard_reply.set_v_4(fpga_.pwrb_io.get_v_buf(4));
  power_dashboard_reply.set_i_4(fpga_.pwrb_io.get_i_buf(4));

  power_dashboard_reply.set_v_5(fpga_.pwrb_io.get_v_buf(5));
  power_dashboard_reply.set_i_5(fpga_.pwrb_io.get_i_buf(5));

  power_dashboard_reply.set_v_6(fpga_.pwrb_io.get_v_buf(6));
  power_dashboard_reply.set_i_6(fpga_.pwrb_io.get_i_buf(6));

  power_dashboard_reply.set_v_7(fpga_.pwrb_io.get_v_buf(7));
  power_dashboard_reply.set_i_7(fpga_.pwrb_io.get_i_buf(7));

  power_dashboard_reply.set_v_8(fpga_.pwrb_io.get_v_buf(8));
  power_dashboard_reply.set_i_8(fpga_.pwrb_io.get_i_buf(8));

  power_dashboard_reply.set_v_9(fpga_.pwrb_io.get_v_buf(9));
  power_dashboard_reply.set_i_9(fpga_.pwrb_io.get_i_buf(9));

  power_dashboard_reply.set_v_10(fpga_.pwrb_io.get_v_buf(10));
  power_dashboard_reply.set_i_10(fpga_.pwrb_io.get_i_buf(10));

  power_dashboard_reply.set_v_11(fpga_.pwrb_io.get_v_buf(11));
  power_dashboard_reply.set_i_11(fpga_.pwrb_io.get_i_buf(11));

  mutex_.unlock();
}

void Kilin::motorStatePack(motor_msg::MotorStateStamped& motor_state_msg) {
  mutex_.lock();
  gettimeofday(&t_stamp, NULL);
  motor_state_msg.mutable_header()->set_seq(seq);
  motor_state_msg.mutable_header()->mutable_stamp()->set_sec(t_stamp.tv_sec);
  motor_state_msg.mutable_header()->mutable_stamp()->set_usec(t_stamp.tv_usec);

  // Helper lambda to convert internal Mode to proto MOTORMODE
  auto modeToProto = [](Mode mode) -> motor_msg::MOTORMODE {
    switch (mode) {
      case Mode::REST: return motor_msg::MOTORMODE::REST_MODE;
      case Mode::CONFIG: return motor_msg::MOTORMODE::CONFIG_MODE;
      case Mode::SET_ZERO: return motor_msg::MOTORMODE::SET_ZERO;
      case Mode::HALL_CALIBRATE: return motor_msg::MOTORMODE::HALL_CALIBRATE;
      case Mode::MOTOR: return motor_msg::MOTORMODE::POSITION_MODE;  // MOTOR maps to POSITION
      case Mode::CONTROL: return motor_msg::MOTORMODE::POSITION_MODE;
      default: return motor_msg::MOTORMODE::REST_MODE;
    }
  };

  // Helper lambda to convert MotorMode to proto MOTORMODE
  auto limbModeToProto = [](MotorMode mode) -> motor_msg::MOTORMODE {
    switch (mode) {
      case MotorMode::REST: return motor_msg::MOTORMODE::REST_MODE;
      case MotorMode::CONFIG: return motor_msg::MOTORMODE::CONFIG_MODE;
      case MotorMode::SET_ZERO: return motor_msg::MOTORMODE::SET_ZERO;
      case MotorMode::HALL_CALIBRATE: return motor_msg::MOTORMODE::HALL_CALIBRATE;
      case MotorMode::POSITION: return motor_msg::MOTORMODE::POSITION_MODE;
      case MotorMode::VELOCITY: return motor_msg::MOTORMODE::VELOCITY_MODE;
      case MotorMode::TORQUE: return motor_msg::MOTORMODE::TORQUE_MODE;
      default: return motor_msg::MOTORMODE::REST_MODE;
    }
  };

  // Module A: LF (Left Front) - index 0
  if (limb_modules_list_.size() > 0) {
    auto* leg_a = motor_state_msg.mutable_module_a();
    
    // Hip motor (LF)
    if (hip_motor_LF_) {
      auto* hip = leg_a->mutable_hip();
      hip->set_position(hip_motor_LF_->rxdata_buffer_.position_);
      hip->set_velocity(hip_motor_LF_->rxdata_buffer_.velocity_);
      hip->set_torque(hip_motor_LF_->rxdata_buffer_.torque_);
      hip->set_motor_mode(modeToProto(hip_motor_LF_->current_mode_));
    }
    
    // Steering and hub motors from limb module 0
    auto& limb_a = limb_modules_list_[0];
    auto* steering = leg_a->mutable_steering();
    steering->set_position(limb_a.steering_motor.pos_act_.as_float);
    steering->set_velocity(limb_a.steering_motor.vel_act_);
    steering->set_torque(limb_a.steering_motor.trq_act_);
    steering->set_motor_mode(limbModeToProto(limb_a.steering_motor.mode_act_));
    
    auto* hub = leg_a->mutable_hub();
    hub->set_position(limb_a.wheel_motor.pos_act_.as_int);
    hub->set_velocity(limb_a.wheel_motor.vel_act_);
    hub->set_torque(limb_a.wheel_motor.trq_act_);
    hub->set_motor_mode(limbModeToProto(limb_a.wheel_motor.mode_act_));
  }

  // Module B: LH (Left Hind) - index 1
  if (limb_modules_list_.size() > 1) {
    auto* leg_b = motor_state_msg.mutable_module_b();
    
    // Hip motor (LH)
    if (hip_motor_LH_) {
      auto* hip = leg_b->mutable_hip();
      hip->set_position(hip_motor_LH_->rxdata_buffer_.position_);
      hip->set_velocity(hip_motor_LH_->rxdata_buffer_.velocity_);
      hip->set_torque(hip_motor_LH_->rxdata_buffer_.torque_);
      hip->set_motor_mode(modeToProto(hip_motor_LH_->current_mode_));
    }
    
    // Steering and hub motors from limb module 1
    auto& limb_b = limb_modules_list_[1];
    auto* steering = leg_b->mutable_steering();
    steering->set_position(limb_b.steering_motor.pos_act_.as_float);
    steering->set_velocity(limb_b.steering_motor.vel_act_);
    steering->set_torque(limb_b.steering_motor.trq_act_);
    steering->set_motor_mode(limbModeToProto(limb_b.steering_motor.mode_act_));
    
    auto* hub = leg_b->mutable_hub();
    hub->set_position(limb_b.wheel_motor.pos_act_.as_int);
    hub->set_velocity(limb_b.wheel_motor.vel_act_);
    hub->set_torque(limb_b.wheel_motor.trq_act_);
    hub->set_motor_mode(limbModeToProto(limb_b.wheel_motor.mode_act_));
  }

  // Module C: RF (Right Front) - index 2
  if (limb_modules_list_.size() > 2) {
    auto* leg_c = motor_state_msg.mutable_module_c();
    
    // Hip motor (RF)
    if (hip_motor_RF_) {
      auto* hip = leg_c->mutable_hip();
      hip->set_position(hip_motor_RF_->rxdata_buffer_.position_);
      hip->set_velocity(hip_motor_RF_->rxdata_buffer_.velocity_);
      hip->set_torque(hip_motor_RF_->rxdata_buffer_.torque_);
      hip->set_motor_mode(modeToProto(hip_motor_RF_->current_mode_));
    }
    
    // Steering and hub motors from limb module 2
    auto& limb_c = limb_modules_list_[2];
    auto* steering = leg_c->mutable_steering();
    steering->set_position(limb_c.steering_motor.pos_act_.as_float);
    steering->set_velocity(limb_c.steering_motor.vel_act_);
    steering->set_torque(limb_c.steering_motor.trq_act_);
    steering->set_motor_mode(limbModeToProto(limb_c.steering_motor.mode_act_));
    
    auto* hub = leg_c->mutable_hub();
    hub->set_position(limb_c.wheel_motor.pos_act_.as_int);
    hub->set_velocity(limb_c.wheel_motor.vel_act_);
    hub->set_torque(limb_c.wheel_motor.trq_act_);
    hub->set_motor_mode(limbModeToProto(limb_c.wheel_motor.mode_act_));
  }

  // Module D: RH (Right Hind) - index 3
  if (limb_modules_list_.size() > 3) {
    auto* leg_d = motor_state_msg.mutable_module_d();
    
    // Hip motor (RH)
    if (hip_motor_RH_) {
      auto* hip = leg_d->mutable_hip();
      hip->set_position(hip_motor_RH_->rxdata_buffer_.position_);
      hip->set_velocity(hip_motor_RH_->rxdata_buffer_.velocity_);
      hip->set_torque(hip_motor_RH_->rxdata_buffer_.torque_);
      hip->set_motor_mode(modeToProto(hip_motor_RH_->current_mode_));
    }
    
    // Steering and hub motors from limb module 3
    auto& limb_d = limb_modules_list_[3];
    auto* steering = leg_d->mutable_steering();
    steering->set_position(limb_d.steering_motor.pos_act_.as_float);
    steering->set_velocity(limb_d.steering_motor.vel_act_);
    steering->set_torque(limb_d.steering_motor.trq_act_);
    steering->set_motor_mode(limbModeToProto(limb_d.steering_motor.mode_act_));
    
    auto* hub = leg_d->mutable_hub();
    hub->set_position(limb_d.wheel_motor.pos_act_.as_int);
    hub->set_velocity(limb_d.wheel_motor.vel_act_);
    hub->set_torque(limb_d.wheel_motor.trq_act_);
    hub->set_motor_mode(limbModeToProto(limb_d.wheel_motor.mode_act_));
  }

  mutex_.unlock();
}

// Convert proto motor mode to internal Mode enum (for HipMotor)
Mode protoToMode(motor_msg::MOTORMODE proto_mode) {
  switch(proto_mode) {
    case motor_msg::MOTORMODE::REST_MODE: return Mode::REST;
    case motor_msg::MOTORMODE::CONFIG_MODE: return Mode::CONFIG;
    case motor_msg::MOTORMODE::SET_ZERO: return Mode::SET_ZERO;
    case motor_msg::MOTORMODE::HALL_CALIBRATE: return Mode::HALL_CALIBRATE;
    case motor_msg::MOTORMODE::POSITION_MODE: return Mode::MOTOR;
    case motor_msg::MOTORMODE::VELOCITY_MODE: return Mode::MOTOR;
    case motor_msg::MOTORMODE::TORQUE_MODE: return Mode::MOTOR;
    default: return Mode::REST;
  }
}

// Convert proto motor mode to limb module mode
MotorMode protoToLimbMode(motor_msg::MOTORMODE proto_mode) {
  switch(proto_mode) {
    case motor_msg::MOTORMODE::REST_MODE: return MotorMode::REST;
    case motor_msg::MOTORMODE::CONFIG_MODE: return MotorMode::CONFIG;
    case motor_msg::MOTORMODE::SET_ZERO: return MotorMode::SET_ZERO;
    case motor_msg::MOTORMODE::HALL_CALIBRATE: return MotorMode::HALL_CALIBRATE;
    case motor_msg::MOTORMODE::POSITION_MODE: return MotorMode::POSITION;
    case motor_msg::MOTORMODE::VELOCITY_MODE: return MotorMode::VELOCITY;
    case motor_msg::MOTORMODE::TORQUE_MODE: return MotorMode::TORQUE;
    default: return MotorMode::REST;
  }
}

void Kilin::motorCommandUnpack(const motor_msg::MotorCmdStamped& motor_cmd_msg) {
  // Note: This function is called with mutex already locked by mainLoop_
  // We mark pending mode changes here, and they will be applied outside the mutex

  // Module A: LF (Left Front) - index 0
  if (motor_cmd_msg.has_module_a()) {
    const auto& leg_a = motor_cmd_msg.module_a();
    
    // Hip motor (LF)
    if (hip_motor_LF_ && leg_a.has_hip()) {
      const auto& hip_cmd = leg_a.hip();
      
      // Update command buffers
      hip_motor_LF_->txdata_buffer_.position_ = hip_cmd.position();
      hip_motor_LF_->txdata_buffer_.torque_ = hip_cmd.torque();
      hip_motor_LF_->txdata_buffer_.KP_ = hip_cmd.kp();
      hip_motor_LF_->txdata_buffer_.KI_ = hip_cmd.ki();
      hip_motor_LF_->txdata_buffer_.KD_ = hip_cmd.kd();
      
      // Mark mode change as pending (will be applied outside mutex)
      Mode new_mode = protoToMode(hip_cmd.motor_mode());
      if (new_mode != hip_motor_LF_->current_mode_) {
        pending_mode_LF_.pending = true;
        pending_mode_LF_.desired_mode = new_mode;
      }
    }
    
    // Steering and hub motors from limb module 0
    if (limb_modules_list_.size() > 0) {
      auto& limb_a = limb_modules_list_[0];
      
      // Steering motor
      if (leg_a.has_steering()) {
        const auto& steering_cmd = leg_a.steering();
        limb_a.steering_motor.mode_des_ = protoToLimbMode(steering_cmd.motor_mode());
        limb_a.steering_motor.pos_des_.as_float = steering_cmd.position();
        limb_a.steering_motor.vel_des_ = steering_cmd.velocity();
        limb_a.steering_motor.trq_des_ = steering_cmd.torque();
      }
      
      // Hub/wheel motor
      if (leg_a.has_hub()) {
        const auto& hub_cmd = leg_a.hub();
        limb_a.wheel_motor.mode_des_ = protoToLimbMode(hub_cmd.motor_mode());
        limb_a.wheel_motor.pos_des_.as_int = hub_cmd.position();
        limb_a.wheel_motor.vel_des_ = hub_cmd.velocity();
        limb_a.wheel_motor.trq_des_ = hub_cmd.torque();
      }
    }
  }

  // Module B: LH (Left Hind) - index 1
  if (motor_cmd_msg.has_module_b()) {
    const auto& leg_b = motor_cmd_msg.module_b();
    
    // Hip motor (LH) // change to RF
    if (hip_motor_RF_ && leg_b.has_hip()) {
      const auto& hip_cmd = leg_b.hip();
      
      hip_motor_RF_->txdata_buffer_.position_ = hip_cmd.position();
      hip_motor_RF_->txdata_buffer_.torque_ = hip_cmd.torque();
      hip_motor_RF_->txdata_buffer_.KP_ = hip_cmd.kp();
      hip_motor_RF_->txdata_buffer_.KI_ = hip_cmd.ki();
      hip_motor_RF_->txdata_buffer_.KD_ = hip_cmd.kd();
      
      Mode new_mode = protoToMode(hip_cmd.motor_mode());
      if (new_mode != hip_motor_RF_->current_mode_) {
        pending_mode_RF_.pending = true;
        pending_mode_RF_.desired_mode = new_mode;
      }
    }

    
    if (limb_modules_list_.size() > 1) {
      auto& limb_b = limb_modules_list_[1];
      
      if (leg_b.has_steering()) {
        const auto& steering_cmd = leg_b.steering();
        limb_b.steering_motor.mode_des_ = protoToLimbMode(steering_cmd.motor_mode());
        limb_b.steering_motor.pos_des_.as_float = steering_cmd.position();
        limb_b.steering_motor.vel_des_ = steering_cmd.velocity();
        limb_b.steering_motor.trq_des_ = steering_cmd.torque();
      }
      
      if (leg_b.has_hub()) {
        const auto& hub_cmd = leg_b.hub();
        limb_b.wheel_motor.mode_des_ = protoToLimbMode(hub_cmd.motor_mode());
        limb_b.wheel_motor.pos_des_.as_int = hub_cmd.position();
        limb_b.wheel_motor.vel_des_ = hub_cmd.velocity();
        limb_b.wheel_motor.trq_des_ = hub_cmd.torque();
      }
    }
  }

  // Module C: RF (Right Front) - index 2
  if (motor_cmd_msg.has_module_c()) {
    const auto& leg_c = motor_cmd_msg.module_c();
    
    // Hip motor (RF) change to LH
    if (hip_motor_LH_ && leg_c.has_hip()) {
      const auto& hip_cmd = leg_c.hip();
      
      hip_motor_LH_->txdata_buffer_.position_ = hip_cmd.position();
      hip_motor_LH_->txdata_buffer_.torque_ = hip_cmd.torque();
      hip_motor_LH_->txdata_buffer_.KP_ = hip_cmd.kp();
      hip_motor_LH_->txdata_buffer_.KI_ = hip_cmd.ki();
      hip_motor_LH_->txdata_buffer_.KD_ = hip_cmd.kd();
      
      Mode new_mode = protoToMode(hip_cmd.motor_mode());
      if (new_mode != hip_motor_LH_->current_mode_) {
        pending_mode_LH_.pending = true;
        pending_mode_LH_.desired_mode = new_mode;
      }
    }

    
    if (limb_modules_list_.size() > 2) {
      auto& limb_c = limb_modules_list_[2];
      
      if (leg_c.has_steering()) {
        const auto& steering_cmd = leg_c.steering();
        limb_c.steering_motor.mode_des_ = protoToLimbMode(steering_cmd.motor_mode());
        limb_c.steering_motor.pos_des_.as_float = steering_cmd.position();
        limb_c.steering_motor.vel_des_ = steering_cmd.velocity();
        limb_c.steering_motor.trq_des_ = steering_cmd.torque();
      }
      
      if (leg_c.has_hub()) {
        const auto& hub_cmd = leg_c.hub();
        limb_c.wheel_motor.mode_des_ = protoToLimbMode(hub_cmd.motor_mode());
        limb_c.wheel_motor.pos_des_.as_int = hub_cmd.position();
        limb_c.wheel_motor.vel_des_ = hub_cmd.velocity();
        limb_c.wheel_motor.trq_des_ = hub_cmd.torque();
      }
    }
  }

  // Module D: RH (Right Hind) - index 3
  if (motor_cmd_msg.has_module_d()) {
    const auto& leg_d = motor_cmd_msg.module_d();
    
    // Hip motor (RH)
    if (hip_motor_RH_ && leg_d.has_hip()) {
      const auto& hip_cmd = leg_d.hip();
      
      hip_motor_RH_->txdata_buffer_.position_ = hip_cmd.position();
      hip_motor_RH_->txdata_buffer_.torque_ = hip_cmd.torque();
      hip_motor_RH_->txdata_buffer_.KP_ = hip_cmd.kp();
      hip_motor_RH_->txdata_buffer_.KI_ = hip_cmd.ki();
      hip_motor_RH_->txdata_buffer_.KD_ = hip_cmd.kd();
      
      Mode new_mode = protoToMode(hip_cmd.motor_mode());
      if (new_mode != hip_motor_RH_->current_mode_) {
        pending_mode_RH_.pending = true;
        pending_mode_RH_.desired_mode = new_mode;
      }
    }
    
    if (limb_modules_list_.size() > 3) {
      auto& limb_d = limb_modules_list_[3];
      
      if (leg_d.has_steering()) {
        const auto& steering_cmd = leg_d.steering();
        limb_d.steering_motor.mode_des_ = protoToLimbMode(steering_cmd.motor_mode());
        limb_d.steering_motor.pos_des_.as_float = steering_cmd.position();
        limb_d.steering_motor.vel_des_ = steering_cmd.velocity();
        limb_d.steering_motor.trq_des_ = steering_cmd.torque();
      }
      
      if (leg_d.has_hub()) {
        const auto& hub_cmd = leg_d.hub();
        limb_d.wheel_motor.mode_des_ = protoToLimbMode(hub_cmd.motor_mode());
        limb_d.wheel_motor.pos_des_.as_int = hub_cmd.position();
        limb_d.wheel_motor.vel_des_ = hub_cmd.velocity();
        limb_d.wheel_motor.trq_des_ = hub_cmd.torque();
      }
    }
  }
}

// Apply pending mode changes outside mutex (uses blocking switch_mode)
void Kilin::applyPendingModeChanges() {
  // Apply mode changes for each motor if pending
  // This function is called outside mutex, so blocking switch_mode() is safe
  
  if (pending_mode_LF_.pending && hip_motor_LF_) {
    hip_motor_LF_->switch_mode(pending_mode_LF_.desired_mode);
    pending_mode_LF_.pending = false;
  }
  
  if (pending_mode_LH_.pending && hip_motor_LH_) {
    hip_motor_LH_->switch_mode(pending_mode_LH_.desired_mode);
    pending_mode_LH_.pending = false;
  }
  
  if (pending_mode_RF_.pending && hip_motor_RF_) {
    hip_motor_RF_->switch_mode(pending_mode_RF_.desired_mode);
    pending_mode_RF_.pending = false;
  }
  
  if (pending_mode_RH_.pending && hip_motor_RH_) {
    hip_motor_RH_->switch_mode(pending_mode_RH_.desired_mode);
    pending_mode_RH_.pending = false;
  }
}


int main(int argc, char* argv[]) {
  signal(SIGINT, inthand);

  important_message("[FPGA Server] : Launched");

  Kilin kilin;

  /* gRPC Topic */
  core::NodeHandler nh;

  core::Publisher<power_msg::PowerStateStamped>& power_pub = nh.advertise<power_msg::PowerStateStamped>("power/state");
  core::Subscriber<power_msg::PowerCmdStamped>& power_sub =
      nh.subscribe<power_msg::PowerCmdStamped>("power/command", 1000, power_data_cb);

  core::Publisher<motor_msg::MotorStateStamped>& motor_pub = nh.advertise<motor_msg::MotorStateStamped>("motor/state");
  core::Subscriber<motor_msg::MotorCmdStamped>& motor_sub =
      nh.subscribe<motor_msg::MotorCmdStamped>("motor/command", 100, motor_data_cb);

  kilin.interruptHandler(power_sub, power_pub, motor_sub, motor_pub);

  if (NiFpga_IsError(kilin.get_fpga_status()))
    std::cout << red << "[FPGA Server] Error! Exiting program. LabVIEW error code: " << kilin.get_fpga_status() << reset
              << std::endl;
  else {
    endwin();
    important_message("\n[FPGA Server] : Exit Safely");
  }
  return 0;
}
