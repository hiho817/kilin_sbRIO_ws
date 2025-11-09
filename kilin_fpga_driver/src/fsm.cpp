#include <fsm.hpp>

ModeFsm::ModeFsm(std::vector<HipMotor>* motors, std::vector<bool>* pb_state) {
  workingMode_ = Mode::REST;
  prev_workingMode_ = Mode::REST;

  hip_motor_list_ = motors;
  pb_state_ = pb_state;

  // Initialize error flag pointers to nullptr (will be set externally)
  NO_CAN_TIMEDOUT_ERROR_ = nullptr;
  NO_SWITCH_TIMEDOUT_ERROR_ = nullptr;

  hall_calibrated = false;
  hall_calibrate_status = 0;
}

void ModeFsm::runFsm(motor_msg::MotorStateStamped& motor_fb_msg,
                     const motor_msg::MotorCmdStamped& motor_cmd_msg) {
  // Skip FSM logic if we don't have hip motors (using individual motor control instead)
  if (!hip_motor_list_) {
    return;
  }
  
  // position = P_CMD_MAX is to make sure the data received from CONFIG function code is the default
  // one
  switch (workingMode_) {
    case Mode::REST: {
      // TODO: pb_state = { digital_switch_, signal_switch_, power_switch_ }
      if (pb_state_ && pb_state_->size() >= 3 && pb_state_->at(2) == true) {
        publishMsg(motor_fb_msg);
        for (auto& motor : *hip_motor_list_) {
          if (motor.enable_) {
            motor.txdata_buffer_.position_ = 0;
            motor.txdata_buffer_.torque_ = 0;
            motor.txdata_buffer_.KP_ = 0;
            motor.txdata_buffer_.KI_ = 0;
            motor.txdata_buffer_.KD_ = 0;
          }
        }
      }
    } break;

    case Mode::SET_ZERO: {
      if (pb_state_ && pb_state_->size() >= 3 && pb_state_->at(2) == true) {
        publishMsg(motor_fb_msg);
        for (auto& motor : *hip_motor_list_) {
          if (motor.enable_) {
            motor.txdata_buffer_.position_ = P_CMD_MAX;
            motor.txdata_buffer_.torque_ = 0;
            motor.txdata_buffer_.KP_ = 0;
            motor.txdata_buffer_.KI_ = 0;
            motor.txdata_buffer_.KD_ = 0;
          }
        }
      }
    } break;

    case Mode::HALL_CALIBRATE: {
      int motor_enabled = 0;

      for (int i = 0; i < 4; i++) {
        if (hip_motor_list_->at(i).enable_) {
          hip_motor_list_->at(i).txdata_buffer_.position_ = 0;
          hip_motor_list_->at(i).txdata_buffer_.torque_ = 0;
          hip_motor_list_->at(i).txdata_buffer_.KP_ = 0;
          hip_motor_list_->at(i).txdata_buffer_.KI_ = 0;
          hip_motor_list_->at(i).txdata_buffer_.KD_ = 0;
          motor_enabled++;
        }
      }
      switch (hall_calibrate_status) {
        case -1: {
          switchMode(Mode::REST);
        } break;

        case 0: {
          int cal_cnt = 0;
          for (int i = 0; i < 4; i++) {
            if (hip_motor_list_->at(i).enable_ &&
                hip_motor_list_->at(i).rxdata_buffer_.cal_stat_ == 2)
              cal_cnt++;
          }
          if (cal_cnt == motor_enabled && measure_offset == 0)
            hall_calibrate_status++;
          else if (cal_cnt == motor_enabled && measure_offset == 1)
            hall_calibrate_status = -1;
        } break;

        case 1: {
          for (int i = 0; i < 4; i++) {
            if (hip_motor_list_->at(i).enable_) {
              hip_motor_list_->at(i).CAN_rx_timedout_ = false;
              hip_motor_list_->at(i).CAN_tx_timedout_ = false;

              cal_command[i] = 0;
              hip_motor_list_->at(i).txdata_buffer_.position_ = 0;
              // Set calibration direction: motors 0 and 2 go positive, motors 1 and 3 go negative
              cal_dir_[i] = (i % 2 == 0) ? 1 : -1;
            }
          }
          hall_calibrate_status++;
        } break;

        case 2: {
          int finish_cnt = 0;
          for (int i = 0; i < 4; i++) {
            if (hip_motor_list_->at(i).enable_) {
              double err = cal_command[i];

              if (fabs(err) < cal_tol_) {
                hip_motor_list_->at(i).txdata_buffer_.position_ = 0;
                hip_motor_list_->at(i).txdata_buffer_.torque_ = 0;
                hip_motor_list_->at(i).txdata_buffer_.KP_ = 0;
                hip_motor_list_->at(i).txdata_buffer_.KI_ = 0;
                hip_motor_list_->at(i).txdata_buffer_.KD_ = 0;
                finish_cnt++;
              } else {
                hip_motor_list_->at(i).io_->set_ni_CAN_id_fc((int)Mode::CONTROL, (int)Mode::CONTROL);
                cal_command[i] += cal_dir_[i] * cal_vel_ * dt_;
                hip_motor_list_->at(i).txdata_buffer_.position_ = cal_command[i];
                hip_motor_list_->at(i).txdata_buffer_.torque_ = 0;
                hip_motor_list_->at(i).txdata_buffer_.KP_ = 50;
                hip_motor_list_->at(i).txdata_buffer_.KI_ = 0;
                hip_motor_list_->at(i).txdata_buffer_.KD_ = 1.5;
              }
            }
          }
          if (finish_cnt == motor_enabled) hall_calibrate_status++;
        } break;

        case 3: {
          hall_calibrated = true;
          hall_calibrate_status = 0;
          switchMode(Mode::MOTOR);
        } break;
      }
    } break;

    case Mode::MOTOR: {
      /* Pubish feedback data from Motors */
      publishMsg(motor_fb_msg);
      
      if (*NO_CAN_TIMEDOUT_ERROR_ && *NO_SWITCH_TIMEDOUT_ERROR_) {
        /*********************************************/
        // Motor Index Mapping:
        //  0: module_a (LF) - Left Front Hip  - CAN0-port0
        //  1: module_d (LH) - Left Hind Hip   - CAN0-port1
        //  2: module_b (RF) - Right Front Hip - CAN1-port0
        //  3: module_c (RH) - Right Hind Hip  - CAN1-port1
        /*********************************************/
        
        for (int i = 0; i < 4; i++) {
          if (hip_motor_list_->at(i).enable_) {
            switch (i) {
              case 0: { // module_a (LF)
                hip_motor_list_->at(i).txdata_buffer_.position_ = motor_cmd_msg.module_a().hip().position();
                hip_motor_list_->at(i).txdata_buffer_.torque_ = motor_cmd_msg.module_a().hip().torque();
                hip_motor_list_->at(i).txdata_buffer_.KP_ = motor_cmd_msg.module_a().hip().kp();
                hip_motor_list_->at(i).txdata_buffer_.KI_ = motor_cmd_msg.module_a().hip().ki();
                hip_motor_list_->at(i).txdata_buffer_.KD_ = motor_cmd_msg.module_a().hip().kd();
              } break;
              
              case 1: { // module_d (LH)
                hip_motor_list_->at(i).txdata_buffer_.position_ = motor_cmd_msg.module_d().hip().position();
                hip_motor_list_->at(i).txdata_buffer_.torque_ = motor_cmd_msg.module_d().hip().torque();
                hip_motor_list_->at(i).txdata_buffer_.KP_ = motor_cmd_msg.module_d().hip().kp();
                hip_motor_list_->at(i).txdata_buffer_.KI_ = motor_cmd_msg.module_d().hip().ki();
                hip_motor_list_->at(i).txdata_buffer_.KD_ = motor_cmd_msg.module_d().hip().kd();
              } break;
              
              case 2: { // module_b (RF)
                hip_motor_list_->at(i).txdata_buffer_.position_ = motor_cmd_msg.module_b().hip().position();
                hip_motor_list_->at(i).txdata_buffer_.torque_ = motor_cmd_msg.module_b().hip().torque();
                hip_motor_list_->at(i).txdata_buffer_.KP_ = motor_cmd_msg.module_b().hip().kp();
                hip_motor_list_->at(i).txdata_buffer_.KI_ = motor_cmd_msg.module_b().hip().ki();
                hip_motor_list_->at(i).txdata_buffer_.KD_ = motor_cmd_msg.module_b().hip().kd();
              } break;
              
              case 3: { // module_c (RH)
                hip_motor_list_->at(i).txdata_buffer_.position_ = motor_cmd_msg.module_c().hip().position();
                hip_motor_list_->at(i).txdata_buffer_.torque_ = motor_cmd_msg.module_c().hip().torque();
                hip_motor_list_->at(i).txdata_buffer_.KP_ = motor_cmd_msg.module_c().hip().kp();
                hip_motor_list_->at(i).txdata_buffer_.KI_ = motor_cmd_msg.module_c().hip().ki();
                hip_motor_list_->at(i).txdata_buffer_.KD_ = motor_cmd_msg.module_c().hip().kd();
              } break;
            }
          }
        }
      }
    } break;

    case Mode::CONFIG: {
      // for debug
    } break;
  }
}

bool ModeFsm::switchMode(Mode next_mode) {
  int mode_switched_cnt = 0;
  int motor_enabled = 0;
  bool success = false;
  Mode next_mode_switch = next_mode;

  for (int i = 0; i < 4; i++) {
    if (hip_motor_list_->at(i).enable_) motor_enabled++;
  }

  double time_elapsed = 0;
  while (1) {
    if (mode_switched_cnt == motor_enabled) {
      prev_workingMode_ = workingMode_;
      workingMode_ = next_mode_switch;
      success = true;
      break;
    } else if (time_elapsed > 3) {
      /* Timeout */
      success = false;
      break;
    } else
      mode_switched_cnt = 0;

    for (int i = 0; i < 4; i++) {
      if (hip_motor_list_->at(i).enable_) {
        // Set mode using switch_mode or directly via io
        uint32_t fc1, fc2;
        hip_motor_list_->at(i).io_->get_ni_CAN_id_fc(&fc1, &fc2);
        // Assuming all motors use the same mode (simplified)
        hip_motor_list_->at(i).io_->set_ni_CAN_id_fc((int)next_mode_switch, (int)next_mode_switch);
        hip_motor_list_->at(i).io_->set_ni_CAN_transmit(true);
        hip_motor_list_->at(i).CAN_receive_feedback();
        if ((next_mode_switch == Mode::SET_ZERO &&
             (int)hip_motor_list_->at(i).rxdata_buffer_.position_ <= 0.01 &&
             (int)hip_motor_list_->at(i).rxdata_buffer_.position_ >= -0.01) ||
            ((int)hip_motor_list_->at(i).rxdata_buffer_.mode_ == (int)next_mode_switch)) {
          mode_switched_cnt++;
        }
      }
    }

    time_elapsed += 0.01;
    usleep(1e4);
  }

  for (int i = 0; i < 4; i++) {
    if (hip_motor_list_->at(i).enable_) {
      if (workingMode_ == Mode::MOTOR)
        hip_motor_list_->at(i).io_->set_ni_CAN_id_fc((int)Mode::CONTROL, (int)Mode::CONTROL);
      else
        hip_motor_list_->at(i).io_->set_ni_CAN_id_fc((int)Mode::CONFIG, (int)Mode::CONFIG);
    }
  }

  return success;
}

void ModeFsm::publishMsg(motor_msg::MotorStateStamped& motor_fb_msg) {
  if (!hip_motor_list_) return;
  
  for (int i = 0; i < 4; i++) {
    if (hip_motor_list_->at(i).enable_) {
      switch (i) {
        case 0:  // module_a (LF)
        {
          motor_fb_msg.mutable_module_a()->mutable_hip()->set_position(
              hip_motor_list_->at(i).rxdata_buffer_.position_);
          motor_fb_msg.mutable_module_a()->mutable_hip()->set_velocity(
              hip_motor_list_->at(i).rxdata_buffer_.velocity_);
          motor_fb_msg.mutable_module_a()->mutable_hip()->set_torque(
              hip_motor_list_->at(i).rxdata_buffer_.torque_ * hip_motor_list_->at(i).txdata_buffer_.KT_);
        } break;

        case 1:  // module_d (LH)
        {
          motor_fb_msg.mutable_module_d()->mutable_hip()->set_position(
              hip_motor_list_->at(i).rxdata_buffer_.position_);
          motor_fb_msg.mutable_module_d()->mutable_hip()->set_velocity(
              hip_motor_list_->at(i).rxdata_buffer_.velocity_);
          motor_fb_msg.mutable_module_d()->mutable_hip()->set_torque(
              hip_motor_list_->at(i).rxdata_buffer_.torque_ * hip_motor_list_->at(i).txdata_buffer_.KT_);
        } break;

        case 2:  // module_b (RF)
        {
          motor_fb_msg.mutable_module_b()->mutable_hip()->set_position(
              hip_motor_list_->at(i).rxdata_buffer_.position_);
          motor_fb_msg.mutable_module_b()->mutable_hip()->set_velocity(
              hip_motor_list_->at(i).rxdata_buffer_.velocity_);
          motor_fb_msg.mutable_module_b()->mutable_hip()->set_torque(
              hip_motor_list_->at(i).rxdata_buffer_.torque_ * hip_motor_list_->at(i).txdata_buffer_.KT_);
        } break;

        case 3:  // module_c (RH)
        {
          motor_fb_msg.mutable_module_c()->mutable_hip()->set_position(
              hip_motor_list_->at(i).rxdata_buffer_.position_);
          motor_fb_msg.mutable_module_c()->mutable_hip()->set_velocity(
              hip_motor_list_->at(i).rxdata_buffer_.velocity_);
          motor_fb_msg.mutable_module_c()->mutable_hip()->set_torque(
              hip_motor_list_->at(i).rxdata_buffer_.torque_ * hip_motor_list_->at(i).txdata_buffer_.KT_);
        } break;
      }
    } else {
      switch (i) {
        case 0:  // module_a (LF)
        {
          motor_fb_msg.mutable_module_a()->mutable_hip()->set_position(0);
          motor_fb_msg.mutable_module_a()->mutable_hip()->set_velocity(0);
          motor_fb_msg.mutable_module_a()->mutable_hip()->set_torque(0);
        } break;

        case 1:  // module_d (LH)
        {
          motor_fb_msg.mutable_module_d()->mutable_hip()->set_position(0);
          motor_fb_msg.mutable_module_d()->mutable_hip()->set_velocity(0);
          motor_fb_msg.mutable_module_d()->mutable_hip()->set_torque(0);
        } break;

        case 2:  // module_b (RF)
        {
          motor_fb_msg.mutable_module_b()->mutable_hip()->set_position(0);
          motor_fb_msg.mutable_module_b()->mutable_hip()->set_velocity(0);
          motor_fb_msg.mutable_module_b()->mutable_hip()->set_torque(0);
        } break;

        case 3:  // module_c (RH)
        {
          motor_fb_msg.mutable_module_c()->mutable_hip()->set_position(0);
          motor_fb_msg.mutable_module_c()->mutable_hip()->set_velocity(0);
          motor_fb_msg.mutable_module_c()->mutable_hip()->set_torque(0);
        } break;
      }
    }
  }
}
