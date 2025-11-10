#ifndef FPGA_SERVER_HPP
#define FPGA_SERVER_HPP

#include <NodeHandler.h>
#include <sys/time.h>
#include <yaml.h>

#include <fstream>
#include <mutex>
#include <string>
#include <vector>

#include "console.hpp"
#include "fpga_handler.hpp"
#include "hip_module.hpp"
#include "hip_motor.hpp"
#include "limb_module.hpp"

// #include <math.h>
// #include <unistd.h>
// #include <Eigen/Dense>

#include "Motor.pb.h"
#include "Power.pb.h"

#ifndef CONFIG_PATH
#define CONFIG_PATH "/home/admin/kilin_sbRIO_ws/kilin_fpga_driver/config/config.yaml"
#endif

void inthand(int signum);
bool is_sys_stop();

class Kilin {
 public:
  Kilin();
  // ~Kilin();

  NiFpga_Status get_fpga_status() { return fpga_.get_fpga_status(); };

  YAML::Node yaml_node_;

  /* console */
  std::mutex main_mtx_;
  Console console_;

  /* interrupt config */
  int main_irq_period_us_;
  int can_irq_period_us_;

  /* header msg */
  struct timeval t_stamp;
  int seq;

  void interruptHandler(core::Subscriber<power_msg::PowerCmdStamped>& cmd_pb_sub_,
                        core::Publisher<power_msg::PowerStateStamped>& state_pb_pub_,
                        core::Subscriber<motor_msg::MotorCmdStamped>& cmd_sub_,
                        core::Publisher<motor_msg::MotorStateStamped>& state_pub_);

  void powerboardPack(power_msg::PowerStateStamped& power_fb_msg);
  void motorStatePack(motor_msg::MotorStateStamped& motor_state_msg);
  void motorCommandUnpack(const motor_msg::MotorCmdStamped& motor_cmd_msg);
  void applyPendingModeChanges();  // Apply mode changes outside mutex
  void rs485Loop_();

 private:
  // Pending mode changes (set by gRPC callback, applied outside mutex)
  struct PendingModeChange {
    bool pending;
    Mode desired_mode;
    PendingModeChange() : pending(false), desired_mode(Mode::REST) {}
  };
  PendingModeChange pending_mode_LF_;
  PendingModeChange pending_mode_LH_;
  PendingModeChange pending_mode_RF_;
  PendingModeChange pending_mode_RH_;
  
  /* robot state */
  // Shared CAN IO objects (one per CAN port)
  ModuleIO_CAN* can_io_L_;  // Shared by LF and LH motors
  ModuleIO_CAN* can_io_R_;  // Shared by RF and RH motors
  
  // Individual hip motors (4 motors: LF, RF, LH, RH)
  HipMotor* hip_motor_LF_;  // Left Front (CAN_L, motor_index 0)
  HipMotor* hip_motor_LH_;  // Left Hind (CAN_L, motor_index 1)
  HipMotor* hip_motor_RF_;  // Right Front (CAN_R, motor_index 0)
  HipMotor* hip_motor_RH_;  // Right Hind (CAN_R, motor_index 1)
  
  std::vector<LimbModule> limb_modules_list_;
  bool HALL_CALIBRATED_;
  int timeout_cnt_;
  int max_timeout_cnt_;

  /* powerboard state */
  std::vector<bool> powerboard_state_;
  bool digital_switch_;
  bool signal_switch_;
  bool power_switch_;
  bool NO_SWITCH_TIMEDOUT_ERROR_;
  bool NO_CAN_TIMEDOUT_ERROR_;

  void load_config_();

  FpgaHandler fpga_;

  void mainLoop_(core::Subscriber<power_msg::PowerCmdStamped>& cmd_pb_sub_,
                 core::Publisher<power_msg::PowerStateStamped>& state_pb_pub_,
                 core::Subscriber<motor_msg::MotorCmdStamped>& cmd_sub_,
                 core::Publisher<motor_msg::MotorStateStamped>& state_pub_);

  void canLoop_();
};

#endif
