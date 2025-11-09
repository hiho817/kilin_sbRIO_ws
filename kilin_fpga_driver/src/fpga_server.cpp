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

  powerboard_state_.push_back(digital_switch_);
  powerboard_state_.push_back(signal_switch_);
  powerboard_state_.push_back(power_switch_);

  // Initialize FSM without hip modules (motors manage their own modes now)
  // ModeFsm fsm(nullptr, &powerboard_state_);
  // fsm_ = fsm;
  // fsm_.NO_CAN_TIMEDOUT_ERROR_ = &NO_CAN_TIMEDOUT_ERROR_;
  // fsm_.NO_SWITCH_TIMEDOUT_ERROR_ = &NO_SWITCH_TIMEDOUT_ERROR_;

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

  // load FSM parameters
  fsm_.dt_ = yaml_node_["MainLoop_period_us"].as<int>() * 0.000001;  // sec
  fsm_.measure_offset = yaml_node_["Measure_offset"].as<int>();
  fsm_.cal_vel_ = yaml_node_["Hall_calibration_vel"].as<double>();
  fsm_.cal_tol_ = yaml_node_["Hall_calibration_tol"].as<double>();

  // load scenario
  if (yaml_node_["Scenario"].as<std::string>().compare("SingleModule") == 0)
    fsm_.scenario_ = Scenario::SINGLE_MODULE;
  else
    fsm_.scenario_ = Scenario::ROBOT;

  // load interrupt periods
  main_irq_period_us_ = yaml_node_["MainLoop_period_us"].as<int>();
  can_irq_period_us_ = yaml_node_["CANLoop_period_us"].as<int>();

  /* Initialize individual hip motors (4 motors total) */
  // Each motor is independent with its own mode control
  // But motors on the same CAN port share the same IO object
  cout << "Initializing individual hip motors..." << endl;

  // Create shared CAN IO objects (one per CAN port)
  can_io_L_ = new ModuleIO_CAN(fpga_.get_fpga_status(), fpga_.session, "MOD1CAN0");
  can_io_R_ = new ModuleIO_CAN(fpga_.get_fpga_status(), fpga_.session, "MOD1CAN1");

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

  // fsm_.runFsm(motor_fb_msg, motor_cmd_data);
  motor_message_updated = 0;
  // HALL_CALIBRATED_ = fsm_.hall_calibrated;

  mutex_.unlock();

  // Communication with Node Architecture
  powerboardPack(power_fb_msg);

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

      // if (power_cmd_data.robot_mode() == (int)Mode::MOTOR && fsm_.workingMode_ != Mode::MOTOR)
      //   fsm_.switchMode(Mode::MOTOR);
      // else if (power_cmd_data.robot_mode() == (int)Mode::HALL_CALIBRATE && fsm_.workingMode_ != Mode::HALL_CALIBRATE &&
      //          fsm_.workingMode_ != Mode::MOTOR)
      //   fsm_.switchMode(Mode::HALL_CALIBRATE);
      // else if (power_cmd_data.robot_mode() == (int)Mode::SET_ZERO && fsm_.workingMode_ != Mode::SET_ZERO)
      //   fsm_.switchMode(Mode::SET_ZERO);
      // else if (power_cmd_data.robot_mode() == (int)Mode::CONFIG && fsm_.workingMode_ != Mode::CONFIG)
      //   fsm_.switchMode(Mode::CONFIG);
      // else if (power_cmd_data.robot_mode() == (int)Mode::REST && fsm_.workingMode_ != Mode::REST)
      //   fsm_.switchMode(Mode::REST);
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
