#ifndef __CONSOLE_H
#define __CONSOLE_H

#define BKGD_PAIR 1
#define CYAN_PAIR 2
#define NCURSES_NOMACROS

#include <locale.h>
#include <ncurses.h>
#include <unistd.h>

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <mode.hpp>
#include <mutex>
#include <string>
#include <thread>

#include "fsm.hpp"
#include "hip_module.hpp"
#undef OK

using namespace std;

class Panel {
 public:
  Panel(string title, string type, HipModule* lm_, int org_x, int org_y, int height_, int width_,
        bool box_on);
  Panel() {}

  WINDOW* win_;

  string title_;

  /* module, control or debug*/
  string type_;
  int org_x_;
  int org_y_;
  int height_;
  int width_;
  bool box_on_;

  HipModule* md_ptr_;
  std::mutex* main_mtx_;
  std::vector<bool>* powerboard_state_;
  ModeFsm* fsm_;

  mutex mutex_;
  void infoDisplay();
  void print_pwrb_info(FpgaHandler* fpga, bool power_switch, bool signal_switch, bool digital_switch);
  void print_mode_main(Behavior bhv, Mode fsm_mode);
  void reset();
  void print_title();
};

class InputPanel {
 public:
  InputPanel() {}

  void init(vector<HipModule>* mods_, bool* if_resetPanel, int term_max_x, int term_max_y);

  void inputHandler(WINDOW* win_, std::mutex& input_mutex);
  void reset_input_window(WINDOW* win);
  void commandDecode(std::string buf);
  vector<string> tokenizer(std::string s);
  auto getValue(string str);

  WINDOW* win_;
  std::mutex mutex_;

  HipModule* modL_ptr_;
  HipModule* modR_ptr_;

  bool* if_resetPanel;
  std::mutex* main_mtx_;
  std::vector<bool>* powerboard_state_;
  ModeFsm* fsm_;

 private:
  std::thread* thread;
};

class Console {
 public:
  Console() {}

  void init(FpgaHandler* fpga_, vector<HipModule>* mods_, std::vector<bool>* pb_state_,
            ModeFsm* fsm_, std::mutex* mtx_);
  void refreshWindow();

  int term_max_x_;
  int term_max_y_;
  int debug_cons_h = 27;
  int power_cons_h = 27;

  FpgaHandler* fpga_;

  Panel p_cmain_;
  Panel p_debug_;
  Panel p_mod1_;
  Panel p_mod2_;
  Panel p_mod3_;
  Panel p_mod4_;
  Panel p_control_;
  InputPanel input_panel_;

  HipModule* modL_ptr_;
  HipModule* modR_ptr_;

  std::mutex* main_mtx_;
  std::vector<bool>* powerboard_state_;
  ModeFsm* fsm_;

  mutex input_mutex_;
  thread t_frontend_;
  int frontend_rate_;

  bool if_resetPanel;
};

#endif
