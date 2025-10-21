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

    inline core::NodeHandler& getNodeHandler() {
      // This object is only created ONCE, the very first time this function is called.
      /**
       * @brief Process-wide singleton instance of core::NodeHandler.
       *
       * This static object serves as the central manager for node registration,
       * lookup, and lifecycle within the FPGA server. It is intended to be the
       * single authoritative NodeHandler shared by all server components.
       *
       * Characteristics:
       * - Lifetime: static storage duration — constructed before first use and
       *   destroyed at program termination (order relative to other statics in
       *   different translation units is unspecified).
       * - Threading: concurrent access must follow the thread-safety guarantees of
       *   core::NodeHandler. If core::NodeHandler is not internally synchronized,
       *   callers must serialize access (for example, with a mutex) to avoid data
       *   races.
       * - Usage: Components should obtain a reference or pointer to this instance
       *   rather than creating additional NodeHandler objects, ensuring a consistent
       *   view of registered nodes and shared resources.
       *
       * Notes:
       * - Avoid relying on static initialization order across translation units.
       *   If deterministic initialization is required, prefer a function-local static
       *   accessor (Meyers' singleton) or explicit initialization sequencing.
       * - If destruction ordering or explicit teardown is necessary, ensure dependent
       *   resources are released before program termination to prevent use-after-destruction.
       */
      static core::NodeHandler instance; 
      return instance;
    }
    // static core::NodeHandler nh;

    NiFpga_Bool get_fpga_status();

    void main_loop(
        core::Subscriber<power_msg::PowerCmdStamped>& pb_cmd_sub_,
        core::Publisher<power_msg::PowerStateStamped>& pb_state_sub_,
        core::Subscriber<motor_msg::MotorCmdStamped>& motor_cmd_sub_,
        core::Publisher<motor_msg::MotorStateStamped>& motor_state_pub_
    );

    static void grpc_motor_sub_cb(motor_msg::MotorCmdStamped motor_msg);
    static void grpc_power_sub_cb(power_msg::PowerCmdStamped power_msg);

  private:
    FpgaHandler fpga_;

    void load_config_();

    void mainLoop_cb_(
      core::Subscriber<power_msg::PowerCmdStamped>& pb_cmd_sub_,
      core::Publisher<power_msg::PowerStateStamped>& pb_state_sub_,
      core::Subscriber<motor_msg::MotorCmdStamped>& motor_cmd_sub_,
      core::Publisher<motor_msg::MotorStateStamped>& motor_state_pub_
    );
    
    void canLoop_cb_();

    YAML::Node yaml_node_;

    /* grpc */
    std::mutex main_mtx_;

    /* console */
    Console console_;

    /* interrupt config */
    int main_irq_period_us_;
    int can_irq_period_us_;

    /* powerboard state */
    std::vector<bool> powerboard_state_;
    bool digital_switch_;
    bool signal_switch_;
    bool power_switch_;
    bool NO_SWITCH_TIMEDOUT_ERROR_;
    bool NO_CAN_TIMEDOUT_ERROR_;

    static bool grpc_hip_motor_cmd_updated_;
    static bool grpc_power_cmd_updated_; // power
    static std::mutex mutex_;

    /* robot state */
    std::vector<HipModule> hip_can_list_;
    ModeFsm fsm_;
    int modules_num_;
    int timeout_cnt_;
    int max_timeout_cnt_;

    /* header msg */
    struct timeval t_stamp;
    int seq;

    void powerboardPack_(power_msg::PowerStateStamped &power_fb_msg);

    static motor_msg::MotorCmdStamped grpc_motor_cmd_data_;
    static power_msg::PowerCmdStamped grpc_power_cmd_data_;
};

#endif
