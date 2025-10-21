#ifndef FPGA_SERVER_HPP
#define FPGA_SERVER_HPP

#include "fpga_handler.hpp"
#include "hip_module.hpp"
#include "console.hpp"
#include "fsm.hpp"

#include <NodeHandler.h>
#include <sys/time.h>
#include <fstream>
#include <yaml.h>
#include <string>
#include <vector>
#include <mutex>

#ifndef CONFIG_PATH
#define CONFIG_PATH "/home/admin/kilin_sbRIO_ws/kilin_fpga_driver/config/config.yaml"
#endif

void inthand(int signum);
bool is_sys_stop();

class Kilin{
  public:
    Kilin();
    // ~Kilin();

    NiFpga_Status get_fpga_status() {return fpga_.get_fpga_status();};

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

    void interruptHandler(
        core::Subscriber<power_msg::PowerCmdStamped>& cmd_pb_sub_,
        core::Publisher<power_msg::PowerStateStamped>& state_pb_pub_,
        core::Subscriber<motor_msg::MotorCmdStamped>& cmd_sub_,
        core::Publisher<motor_msg::MotorStateStamped>& state_pub_);

    void powerboardPack(power_msg::PowerStateStamped &power_fb_msg);
    
  private:
    /* robot state */
    std::vector<HipModule> modules_list_;
    ModeFsm fsm_;
    bool HALL_CALIBRATED_;
    int modules_num_;
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

    void mainLoop_(
        core::Subscriber<power_msg::PowerCmdStamped>& cmd_pb_sub_,
        core::Publisher<power_msg::PowerStateStamped>& state_pb_pub_,
        core::Subscriber<motor_msg::MotorCmdStamped>& cmd_sub_,
        core::Publisher<motor_msg::MotorStateStamped>& state_pub_);
    
    void canLoop_();
};

#endif
