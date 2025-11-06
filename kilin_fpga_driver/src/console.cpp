#include "console.hpp"

#include <curses.h>

#include <iterator>

using namespace std;

mutex cons_mtx_;
int refresh_flag;

void Console::init(FpgaHandler* fpga, vector<HipModule>* hip_mods, vector<LimbModule>* limb_mods,
                   std::vector<bool>* pb_state_ptr_, ModeFsm* fsm_ptr_, std::mutex* mtx_ptr_) {
  fpga_ = fpga;

  modL_ptr_ = &hip_mods->at(0);
  modR_ptr_ = &hip_mods->at(1);

  limb_modules_ptr_ = limb_mods;

  setlocale(LC_ALL, "");
  initscr();
  getmaxyx(stdscr, term_max_y_, term_max_x_);
  start_color();
  init_pair(BKGD_PAIR, COLOR_WHITE, COLOR_BLACK);
  wbkgd(stdscr, COLOR_PAIR(BKGD_PAIR));
  init_pair(CYAN_PAIR, COLOR_CYAN, COLOR_BLACK);

  frontend_rate_ = 3;
  input_panel_.init(hip_mods, limb_modules_ptr_, &if_resetPanel, term_max_x_, term_max_y_);

  input_panel_.main_mtx_ = mtx_ptr_;
  input_panel_.powerboard_state_ = pb_state_ptr_;
  input_panel_.fsm_ = fsm_ptr_;

  main_mtx_ = mtx_ptr_;
  powerboard_state_ = pb_state_ptr_;
  fsm_ = fsm_ptr_;

  if_resetPanel = false;
  t_frontend_ = thread(&Console::refreshWindow, this);
  refresh_flag = 1;
}

void Console::refreshWindow() {
  clear();

  int refresh_period_ = (int)(1 / frontend_rate_) * 1000000;
  HipModule* lm_null = 0;
  Panel p_power_("[P] Power Board ", "power", lm_null, 1, 9, 60, 40, true);
  Panel p_cmain_("[F] FPGA Server ", "c_main", lm_null, 1, 1, 8, 40, true);
  Panel p_modL_("[L] L_Module ", "module", modL_ptr_, 41, 1, (term_max_y_ - 2) / 2 - 1, 60, true);
  Panel p_modR_("[R] R_Module ", "module", modR_ptr_, 41, (term_max_y_) / 2, (term_max_y_ - 2) / 2 - 1, 60, true);
  Panel p_mod_test_RS485("[T] Test Limb ", "RS485_test", lm_null, 95, 1, term_max_y_, 100, true);
  // Panel p_modS1_("[1] Steering1 ", "module", modL_ptr_, 93, 1, (term_max_y_ - 2) / 2 - 1, 60,
  // true);

  p_power_.powerboard_state_ = powerboard_state_;
  p_cmain_.fsm_ = fsm_;

  while (1) {
    cons_mtx_.lock();

    p_power_.print_pwrb_info(fpga_, powerboard_state_->at(0), powerboard_state_->at(1), powerboard_state_->at(2));
    p_cmain_.print_mode_main(Behavior::TCP_SLAVE, fsm_->workingMode_);
    p_modL_.infoDisplay();
    p_modR_.infoDisplay();
    p_mod_test_RS485.print_limb_test(fpga_, limb_modules_ptr_);
    // p_modS1_.infoDisplay();
    cons_mtx_.unlock();

    usleep(0.1 * 1000 * 1000);
  }
}

void InputPanel::init(vector<HipModule>* hip_mods, vector<LimbModule>* limb_mods, bool* if_resetPanel, int term_max_x, int term_max_y) {
  win_ = newwin(3, term_max_x - 1, term_max_y - 3, 1);

  modL_ptr_ = &hip_mods->at(0);
  modR_ptr_ = &hip_mods->at(1);

  limb_modules_ptr_ = limb_mods;

  thread = new std::thread(&InputPanel::inputHandler, this, win_, std::ref(mutex_));
}

void InputPanel::inputHandler(WINDOW* win_, std::mutex& input_mutex) {
  while (1) {
    int ch = 0;
    int x = 1;
    do {
      ch = mvwgetch(win_, 1, x);
      if (ch == 'r') {
        reset_input_window(win_);
      }
      if (ch == 'e') {
        endwin();
        std::cout << "Normal Mode" << std::endl;
        refresh_flag = 0;
      }
      if (ch == 'E') {
        refresh_flag = 1;
        refresh();
      }
      // if (ch == 'R') {
      //   for (size_t i = 0; i < limb_modules_ptr_->size(); i++) {
      //       limb_modules_ptr_->at(i).update_motors();
      //   }
      // }

    } while (ch != ':');

    string input_buf;
    keypad(win_, true);
    do {
      ch = mvwgetch(win_, 1, x);

      if (ch == KEY_BACKSPACE || ch == KEY_DC || ch == 127) {
        mvwdelch(win_, 1, x);
        mvwdelch(win_, 1, x + 1);
        mvwdelch(win_, 1, x - 1);
        wclrtoeol(win_);
        wrefresh(win_);

        if (input_buf.size() > 0) {
          input_buf.erase(input_buf.size() - 1, 1);
          x--;
        }
      } else {
        if (ch != '\n') {
          input_buf.append(1, ch);
        }
        x++;
      }
    } while (ch != '\n');

    reset_input_window(win_);
    commandDecode(input_buf);
  }
}

void InputPanel::reset_input_window(WINDOW* win) {
  werase(win);
  wclear(win);
  wrefresh(win);
}

void InputPanel::commandDecode(string buf) {
  bool syntax_err = false;
  bool pb_selected = false;
  bool lm_selected = false;
  bool f_selected = false;
  bool limb_selected = false;

  bool switchFSM_success = true;
  HipModule* md_ptr_;

  vector<string> bufs;
  bufs = tokenizer(buf);

  if (bufs.size() >= 1) {
    if (bufs[0] == "P") {
      pb_selected = true;
    } else if (bufs[0] == "F") {
      f_selected = true;
    } else if (bufs[0] == "L") {
      lm_selected = true;
      md_ptr_ = modL_ptr_;
    } else if (bufs[0] == "R") {
      lm_selected = true;
      md_ptr_ = modR_ptr_;
    } else if (bufs[0] == "T") {
      limb_selected = true;
    } else {
      syntax_err = true;
    }

    mvwprintw(win_, 2, 1, bufs[0].c_str());
    wrefresh(win_);
  }

  cons_mtx_.lock();
  main_mtx_->lock();

  if (bufs.size() == 3) {
    mvwprintw(win_, 2, 3, bufs[1].c_str());
    mvwprintw(win_, 2, 5, bufs[2].c_str());
    wrefresh(win_);

    if (pb_selected) {
      if (bufs[1] == "D") {
        try {
          powerboard_state_->at(0) = stoi(bufs[2]);
        } catch (exception& e) {
          syntax_err = true;
          mvwprintw(win_, 2, 1, "err");
        }
      } else if (bufs[1] == "S") {
        try {
          powerboard_state_->at(1) = stoi(bufs[2]);
        } catch (exception& e) {
          syntax_err = true;
          mvwprintw(win_, 2, 1, "err");
        }
      } else if (bufs[1] == "P") {
        try {
          powerboard_state_->at(2) = stoi(bufs[2]);
        } catch (exception& e) {
          syntax_err = true;
          mvwprintw(win_, 2, 1, "err");
        }
      } else {
        syntax_err = true;
      }
    } else if (f_selected) {
      if (bufs[1] == "M") {
        if (bufs[2] == "R") {
          switchFSM_success = fsm_->switchMode(Mode::REST);
        } else if (bufs[2] == "M") {
          switchFSM_success = fsm_->switchMode(Mode::MOTOR);
        } else if (bufs[2] == "S") {
          switchFSM_success = fsm_->switchMode(Mode::SET_ZERO);
        } else if (bufs[2] == "H") {
          switchFSM_success = fsm_->switchMode(Mode::HALL_CALIBRATE);
        } else {
          syntax_err = true;
        }
      } else {
        syntax_err = true;
      }
    } else {
      syntax_err = true;
    }
  } else if (bufs.size() == 4) {
    mvwprintw(win_, 2, 3, bufs[1].c_str());
    mvwprintw(win_, 2, 5, bufs[2].c_str());
    mvwprintw(win_, 2, 7, bufs[3].c_str());
    wrefresh(win_);

    int mtr_idx = 0;
    if (bufs[1] == "F") {
      mtr_idx = 0;
    } else if (bufs[1] == "H") {
      mtr_idx = 1;
    } else {
      syntax_err = true;
    }

    if (!syntax_err) {
      if (bufs[2] == "C") {
        try {
          md_ptr_->motors_list_[mtr_idx].CAN_ID_ = stoi(bufs[3]);
        } catch (exception& e) {
          syntax_err = true;
          mvwprintw(win_, 2, 1, "syntax error");
        }
      } else if (bufs[2] == "A") {
        try {
          md_ptr_->txdata_buffer_[mtr_idx].position_ = stof(bufs[3]);
        } catch (exception& e) {
          syntax_err = true;
        }
      } else if (bufs[2] == "T") {
        try {
          md_ptr_->txdata_buffer_[mtr_idx].torque_ = stof(bufs[3]);
        } catch (exception& e) {
          syntax_err = true;
        }
      } else if (bufs[2] == "P") {
        try {
          md_ptr_->txdata_buffer_[mtr_idx].KP_ = stof(bufs[3]);
        } catch (exception& e) {
          syntax_err = true;
        }
      } else if (bufs[2] == "I") {
        try {
          md_ptr_->txdata_buffer_[mtr_idx].KI_ = stof(bufs[3]);
        } catch (exception& e) {
          syntax_err = true;
        }
      } else if (bufs[2] == "D") {
        try {
          md_ptr_->txdata_buffer_[mtr_idx].KD_ = stof(bufs[3]);
        } catch (exception& e) {
          syntax_err = true;
        }
      } else {
        syntax_err = true;
      }
    }
  } else if (bufs.size() == 5) {
    // Format: T <module_id> <motor> <command> <value>
    // T 0 S M 4  -> Set limb module 0, steering motor, mode to 4 (POSITION)
    // T 0 S P 100 -> Set limb module 0, steering motor, position to 100
    // T 0 W V 50 -> Set limb module 0, wheel motor, velocity to 50
    mvwprintw(win_, 2, 3, bufs[1].c_str());
    mvwprintw(win_, 2, 5, bufs[2].c_str());
    mvwprintw(win_, 2, 7, bufs[3].c_str());
    mvwprintw(win_, 2, 9, bufs[4].c_str());
    wrefresh(win_);
    
    if (limb_selected) {
      try {
        int module_idx = stoi(bufs[1]);
        
        if (module_idx < 0 || module_idx >= (int)limb_modules_ptr_->size()) {
          syntax_err = true;
          mvwprintw(win_, 2, 1, "Invalid module index");
        } else {
          LimbModule& limb_mod = limb_modules_ptr_->at(module_idx);
          
          // Determine motor (S=steering, W=wheel)
          bool is_steering = (bufs[2] == "S");
          bool is_wheel = (bufs[2] == "W");
          
          if (!is_steering && !is_wheel) {
            syntax_err = true;
            mvwprintw(win_, 2, 1, "Invalid motor (use S or W)");
          } else {
            // Determine command
            if (bufs[3] == "M") {
              // Set mode: R=0(REST), C=1(CONFIG), Z=2(SET_ZERO), H=3(HALL_CAL), P=4(POSITION), V=5(VELOCITY), T=6(TORQUE)
              int mode_val = stoi(bufs[4]);
              if (mode_val >= 0 && mode_val <= 6) {
                if (is_steering) {
                  limb_mod.steering_motor.mode_des_ = static_cast<MotorMode>(mode_val);
                } else {
                  limb_mod.wheel_motor.mode_des_ = static_cast<MotorMode>(mode_val);
                }
              } else {
                syntax_err = true;
                mvwprintw(win_, 2, 1, "Invalid mode value");
              }
            } else if (bufs[3] == "P") {
              // Set position
              double pos_val = stof(bufs[4]);
              if (is_steering) {
                limb_mod.set_steering_position(pos_val);
              } else {
                // Wheel doesn't typically use position control, but we can set the desired value
                limb_mod.wheel_motor.mode_des_ = MotorMode::POSITION;
                limb_mod.wheel_motor.pos_des_.as_int = static_cast<int32_t>(pos_val);
              }
            } else if (bufs[3] == "V") {
              // Set velocity
              double vel_val = stof(bufs[4]);
              if (is_steering) {
                limb_mod.set_steering_velocity(vel_val);
              } else {
                limb_mod.set_wheel_velocity(vel_val);
              }
            } else if (bufs[3] == "T") {
              // Set torque
              double trq_val = stof(bufs[4]);
              if (is_steering) {
                limb_mod.set_steering_torque(trq_val);
              } else {
                limb_mod.set_wheel_torque(trq_val);
              }
            } else {
              syntax_err = true;
              mvwprintw(win_, 2, 1, "Invalid command");
            }
          }
        }
      } catch (exception& e) {
        syntax_err = true;
        mvwprintw(win_, 2, 1, "Parse error");
      }
    } else {
      syntax_err = true;
    }
  } else {
    syntax_err = true;
  }

  if (syntax_err) {
    mvwprintw(win_, 0, 1, "Syntax Error !");
  } else if (!switchFSM_success) {
    mvwprintw(win_, 0, 1, "Switch Mode Timeout !");
  } else {
    mvwprintw(win_, 0, 1, "Command Send !");
  }

  cons_mtx_.unlock();
  main_mtx_->unlock();

  wrefresh(win_);
}

vector<string> InputPanel::tokenizer(string s) {
  // A quick way to split strings separated via spaces.
  stringstream ss(s);
  string word;
  vector<string> bufs;
  while (ss >> word) {
    // cout << word << endl;
    bufs.push_back(word);
  }
  return bufs;
}

Panel::Panel(string title, string type, HipModule* lm_, int org_x, int org_y, int height, int width, bool box_on) {
  org_x_ = org_x;
  org_y_ = org_y;
  height_ = height;
  width_ = width;
  // box_on_ = // box_on;
  type_ = type;
  title_ = title;

  md_ptr_ = lm_;

  win_ = newwin(height_, width_, org_y_, org_x_);
  refresh();

  // box(win_, 0, 0);

  string tag_(title.c_str(), title.c_str() + 3);
  title.erase(0, 3);

  // mvwprintw(win_, 0, (width_ / 2 - (title.size() + tag_.size()) / 2), tag_.c_str());
  wattron(win_, COLOR_PAIR(CYAN_PAIR));
  wattron(win_, A_BOLD);
  wattron(win_, A_STANDOUT);
  mvwprintw(win_, 0, (width_ / 2 - title.size() / 2 - 2), tag_.c_str());
  wattroff(win_, COLOR_PAIR(CYAN_PAIR));

  mvwprintw(win_, 0, (width_ / 2 - title.size() / 2 + 1), title.c_str());
  wattroff(win_, A_BOLD);
  wattroff(win_, A_STANDOUT);
  wrefresh(win_);
  // refresh();
}

void Panel::infoDisplay()  // type = module
{
  int y_org = 2;

  // Motor_F
  mvwprintw(win_, 1, 1, "[F] Motor_F---------------------------------------");
  mvwprintw(win_, 2, 1, "[C] [CAN] ID: %9d", md_ptr_->motors_list_[0].CAN_ID_);
  mvwprintw(win_, 3, 1, "    [tx] TIMEDOUT: %4d", md_ptr_->CAN_tx_timedout_[0]);
  mvwprintw(win_, y_org + 2, 1, "[A] [tx] Pos:  %5.5f", md_ptr_->txdata_buffer_[0].position_);
  mvwprintw(win_, y_org + 3, 1, "[T] [tx] Trq:  %5.5f", md_ptr_->txdata_buffer_[0].torque_);
  mvwprintw(win_, y_org + 4, 1, "[P] [tx] KP:   %4.5f", md_ptr_->txdata_buffer_[0].KP_);
  mvwprintw(win_, y_org + 5, 1, "[I] [tx] KI:   %5.5f", md_ptr_->txdata_buffer_[0].KI_);
  mvwprintw(win_, y_org + 6, 1, "[D] [tx] KD:   %5.5f", md_ptr_->txdata_buffer_[0].KD_);
  // reply
  mvwprintw(win_, 3, 30, "[rx] TIMEDOUT: %4d", md_ptr_->CAN_rx_timedout_[0]);
  mvwprintw(win_, y_org + 2, 30, "[rx] Ver:   %7d", md_ptr_->rxdata_buffer_[0].version_);
  mvwprintw(win_, y_org + 3, 30, "[rx] Mode:  %7d", md_ptr_->rxdata_buffer_[0].mode_state_);
  mvwprintw(win_, y_org + 4, 30, "[rx] Pos:   %4.5f", md_ptr_->rxdata_buffer_[0].position_);
  mvwprintw(win_, y_org + 5, 30, "[rx] Vel:   %4.5f", md_ptr_->rxdata_buffer_[0].velocity_);
  mvwprintw(win_, y_org + 6, 30, "[rx] Trq:   %4.5f", md_ptr_->rxdata_buffer_[0].torque_);
  mvwprintw(win_, y_org + 7, 30, "[rx] Cal:   %7d", md_ptr_->rxdata_buffer_[0].calibrate_finish_);

  // Motor_H
  mvwprintw(win_, 10, 1, "[H] Motor_H---------------------------------------");
  mvwprintw(win_, 11, 1, "[C] [CAN] ID: %9d", md_ptr_->motors_list_[1].CAN_ID_);
  mvwprintw(win_, 12, 1, "    [tx] TIMEDOUT: %4d", md_ptr_->CAN_tx_timedout_[1]);
  mvwprintw(win_, y_org + 11, 1, "[A] [tx] Pos:  %5.5f", md_ptr_->txdata_buffer_[1].position_);
  mvwprintw(win_, y_org + 12, 1, "[T] [tx] Trq:  %5.5f", md_ptr_->txdata_buffer_[1].torque_);
  mvwprintw(win_, y_org + 13, 1, "[P] [tx] KP:   %4.5f", md_ptr_->txdata_buffer_[1].KP_);
  mvwprintw(win_, y_org + 14, 1, "[I] [tx] KI:   %5.5f", md_ptr_->txdata_buffer_[1].KI_);
  mvwprintw(win_, y_org + 15, 1, "[D] [tx] KD:   %5.5f", md_ptr_->txdata_buffer_[1].KD_);
  // reply
  mvwprintw(win_, 12, 30, "[rx] TIMEDOUT: %4d", md_ptr_->CAN_rx_timedout_[1]);
  mvwprintw(win_, y_org + 11, 30, "[rx] Ver:   %7d", md_ptr_->rxdata_buffer_[1].version_);
  mvwprintw(win_, y_org + 12, 30, "[rx] Mode:  %7d", md_ptr_->rxdata_buffer_[1].mode_state_);
  mvwprintw(win_, y_org + 13, 30, "[rx] Pos:   %4.5f", md_ptr_->rxdata_buffer_[1].position_);
  mvwprintw(win_, y_org + 14, 30, "[rx] Vel:   %4.5f", md_ptr_->rxdata_buffer_[1].velocity_);
  mvwprintw(win_, y_org + 15, 30, "[rx] Trq:   %4.5f", md_ptr_->rxdata_buffer_[1].torque_);
  mvwprintw(win_, y_org + 16, 30, "[rx] Cal:   %7d", md_ptr_->rxdata_buffer_[1].calibrate_finish_);
  wrefresh(win_);
}

void Panel::print_limb_test(FpgaHandler* fpga, vector<LimbModule>* limb_mods) {
  mvwprintw(win_, 1, 1, "================================== RS485 Debug Panel ==================================");
  
  int line = 2;
  for (size_t i = 0; i < limb_mods->size(); i++) {
    LimbModule& module = limb_mods->at(i);
    
    // Module header with communication status
    mvwprintw(win_, line++, 1, ">> Module [%zu]: %s  |  Port:%d  |  Status:%s  |  FPGA:%s  |  TX:%d RX:%d CKS:%d", i, 
              module.label_.c_str(),
              module.rs485_port_,
              module.is_communication_ok() ? "OK     " : "TIMEOUT",
              module.io_.get_fpga_status() ? "ERR" : "OK",
              module.io_.get_ni_tx_count(),
              module.io_.get_ni_rx_count(),
              module.io_.get_ni_checksum_ok());
    
    // TX and RX Buffer on same lines (compact)
    mvwprintw(win_, line++, 1, "   TX> M1: %02X %02X %08X | M2: %02X %02X %08X",
              module.txdata_buffer_.CMD1, module.txdata_buffer_.SUBCMD1, module.txdata_buffer_.Data1,
              module.txdata_buffer_.CMD2, module.txdata_buffer_.SUBCMD2, module.txdata_buffer_.Data2);
    
    mvwprintw(win_, line++, 1, "   RX> M1: %02X %08X %02X %08X | M2: %02X %08X %02X %08X",
              module.rxdata_buffer_.CMD1, module.rxdata_buffer_.POS1, module.rxdata_buffer_.STAT1, module.rxdata_buffer_.I1,
              module.rxdata_buffer_.CMD2, module.rxdata_buffer_.POS2, module.rxdata_buffer_.STAT2, module.rxdata_buffer_.I2);
    
    // Steering motor: Desired vs Actual
    mvwprintw(win_, line++, 1, "   Steer | Des[M:%d]: P=%10.3f V=%5.3f T=%3.3f | Act[M:%d]: P=%10.3f V=%5.3f T=%3.3f", 
              (int)module.steering_motor.mode_des_,
              module.steering_motor.pos_des_.as_float, module.steering_motor.vel_des_, module.steering_motor.trq_des_,
              (int)module.steering_motor.mode_act_,
              module.steering_motor.pos_act_.as_float, module.steering_motor.vel_act_, module.steering_motor.trq_act_);
    
    // Wheel motor: Desired vs Actual
    mvwprintw(win_, line++, 1, "   Wheel | Des[M:%d]: P=%10d V=%5.3f T=%3.3f | Act[M:%d]: P=%10d V=%5.3f T=%3.3f",
              (int)module.wheel_motor.mode_des_,
              module.wheel_motor.pos_des_.as_int, module.wheel_motor.vel_des_, module.wheel_motor.trq_des_,
              (int)module.wheel_motor.mode_act_,
              module.wheel_motor.pos_act_.as_int, module.wheel_motor.vel_act_, module.wheel_motor.trq_act_);
    
    // Mode change management debug info
    mvwprintw(win_, line++, 1, "   Mode> Steer: PrevM:%d ChgSent:%d | Wheel: PrevM:%d ChgSent:%d",
              (int)module.get_prev_mode_des_steering(), module.get_mode_change_sent_steering(),
              (int)module.get_prev_mode_des_wheel(), module.get_mode_change_sent_wheel());
    
    // Module separator (compact)
    if (i < limb_mods->size() - 1) {
      mvwprintw(win_, line++, 1, "   -------------------------------------------------------------------------------");
    }
  }

  wrefresh(win_);
}

// type = power
void Panel::print_pwrb_info(FpgaHandler* fpga, bool digital_switch, bool signal_switch, bool power_switch) {
  mvwprintw(win_, 2, 1, "HARDWARE POWER SWITCH ----------------");
  mvwprintw(win_, 3, 1, "[D] Digital:   %4d", digital_switch);
  mvwprintw(win_, 4, 1, "[S] Signal:    %4d", signal_switch);
  mvwprintw(win_, 5, 1, "[P] Power:     %4d", power_switch);
  mvwprintw(win_, 6, 1, "Voltage Current ADC ------------------");
  mvwprintw(win_, 7, 1, "Voltage: %5.5f, Current: %5.5f", fpga->pwrb_io.get_v_buf(0), fpga->pwrb_io.get_i_buf(0));
  mvwprintw(win_, 8, 1, "Voltage: %5.5f, Current: %5.5f", fpga->pwrb_io.get_v_buf(1), fpga->pwrb_io.get_i_buf(1));
  mvwprintw(win_, 9, 1, "Voltage: %5.5f, Current: %5.5f", fpga->pwrb_io.get_v_buf(2), fpga->pwrb_io.get_i_buf(2));
  mvwprintw(win_, 10, 1, "Voltage: %5.5f, Current: %5.5f", fpga->pwrb_io.get_v_buf(3), fpga->pwrb_io.get_i_buf(3));
  mvwprintw(win_, 11, 1, "Voltage: %5.5f, Current: %5.5f", fpga->pwrb_io.get_v_buf(4), fpga->pwrb_io.get_i_buf(4));
  mvwprintw(win_, 12, 1, "Voltage: %5.5f, Current: %5.5f", fpga->pwrb_io.get_v_buf(5), fpga->pwrb_io.get_i_buf(5));
  mvwprintw(win_, 13, 1, "Voltage: %5.5f, Current: %5.5f", fpga->pwrb_io.get_v_buf(6), fpga->pwrb_io.get_i_buf(6));
  mvwprintw(win_, 14, 1, "Voltage: %5.5f, Current: %5.5f", fpga->pwrb_io.get_v_buf(7), fpga->pwrb_io.get_i_buf(7));
  mvwprintw(win_, 15, 1, "Voltage: %5.5f, Current: %5.5f", fpga->pwrb_io.get_v_buf(8), fpga->pwrb_io.get_i_buf(8));
  mvwprintw(win_, 16, 1, "Voltage: %5.5f, Current: %5.5f", fpga->pwrb_io.get_v_buf(9), fpga->pwrb_io.get_i_buf(9));
  mvwprintw(win_, 17, 1, "Voltage: %5.5f, Current: %5.5f", fpga->pwrb_io.get_v_buf(10), fpga->pwrb_io.get_i_buf(10));
  mvwprintw(win_, 18, 1, "Voltage: %5.5f, Current: %5.5f", fpga->pwrb_io.get_v_buf(11), fpga->pwrb_io.get_i_buf(11));

  // for (int i = 0; i < 12; ++i) {
  //   mvwprintw(win_, 7 + i, 1, "Voltage: %5.5f, Current: %5.5f", fpga->pwrb_io.get_v_buf(i),
  //   fpga->pwrb_io.get_i_buf(i));
  // }

  wrefresh(win_);
}

// type = c_main
void Panel::print_mode_main(Behavior bhv, Mode fsm_mode) {
  if (bhv == Behavior::TCP_SLAVE)
    mvwprintw(win_, 2, 1, "Behavior: TCP_SLAVE");
  else if (bhv == Behavior::SET_THETA)
    mvwprintw(win_, 2, 1, "Behavior: SET_THETA");
  else if (bhv == Behavior::CUSTOM_1)
    mvwprintw(win_, 2, 1, "Behavior: CUSTOM_1");
  else if (bhv == Behavior::CUSTOM_1)
    mvwprintw(win_, 2, 1, "Behavior: CUSTOM_2");
  else if (bhv == Behavior::CUSTOM_1)
    mvwprintw(win_, 2, 1, "Behavior: CUSTOM_3");

  if (fsm_mode == Mode::REST)
    mvwprintw(win_, 3, 1, "[M] FSM Mode:           REST");
  else if (fsm_mode == Mode::SET_ZERO)
    mvwprintw(win_, 3, 1, "[M] FSM Mode:       SET_ZERO");
  else if (fsm_mode == Mode::HALL_CALIBRATE)
    mvwprintw(win_, 3, 1, "[M] FSM Mode: HALL_CALIBRATE");
  else if (fsm_mode == Mode::MOTOR)
    mvwprintw(win_, 3, 1, "[M] FSM Mode:          MOTOR");
  mvwprintw(win_, 5, 1, "[R] REST  [S] SET_ZERO ");
  mvwprintw(win_, 6, 1, "[M] MOTOR [H] HALL_CALIBRATE");

  wrefresh(win_);
}

void Panel::print_title() {
  string tag_(title_.c_str(), title_.c_str() + 3);
  title_.erase(0, 3);
  wattron(win_, COLOR_PAIR(CYAN_PAIR));
  wattron(win_, A_BOLD);
  wattron(win_, A_STANDOUT);
  mvwprintw(win_, 0, (width_ / 2 - title_.size() / 2 - 2), tag_.c_str());
  wattroff(win_, COLOR_PAIR(CYAN_PAIR));

  mvwprintw(win_, 0, (width_ / 2 - title_.size() / 2 + 1), title_.c_str());
  wattroff(win_, A_BOLD);
  wattroff(win_, A_STANDOUT);

  wrefresh(win_);
}

void Panel::reset() {
  werase(win_);
  wclear(win_);
  wrefresh(win_);
}
