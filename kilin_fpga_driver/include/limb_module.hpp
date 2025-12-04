#ifndef __LIMBMODULE_H
#define __LIMBMODULE_H

#include <math.h>
#include <yaml-cpp/yaml.h>

#include <eigen3/Eigen/Dense>
#include <iomanip>
#include <iostream>
#include <vector>

#include "fpga_handler.hpp"
#include "msg.hpp"

/*= Servo CMD =*/
enum ServoCmd : uint8_t {
  CMD_RESET = 0x00,
  CMD_CONFIG = 0x01,
  CMD_SET_ZERO = 0x02,
  CMD_HAL_CAL = 0x03,     // Reserved
  CMD_MOTOR_MODE = 0x04,  // this is to set mode (pos or vel or torque)
  CMD_MOTOR_CMD = 0x05,   // this is the one that makes the motor move
  CMD_ECHO = 0x06         // Echo mode - used after sending calibration command
};

/*= Servo Sub CMD =*/
constexpr uint8_t SHIFTBIT_SUBCMD = 4;

enum ServoSubCmd : uint8_t {
  // // SUBCMD for CMD_CONFIG of steering motor
  // SUBCMD_CONFIG_READ = 0x00,
  // SUBCMD_CONFIG_WRITE = 0x01,

  // SUBCMD for CMD_MOTOR_MODE of wheelhub motor
  SUBCMD_MOTOR_POSITON = 0x00,
  SUBCMD_MOTOR_SPEED = 0x01,
  SUBCMD_MOTOR_TORQUE = 0x02
};

// TX Data Buffer with union for easy byte access
union RS485_txdata_buf {
  struct __attribute__((packed)) {
    ServoCmd CMD1;        // Byte 0
    ServoSubCmd SUBCMD1;  // Byte 1
    int32_t Data1;        // Bytes 2-5
    ServoCmd CMD2;        // Byte 6
    ServoSubCmd SUBCMD2;  // Byte 7
    int32_t Data2;        // Bytes 8-11
  };
  uint8_t bytes[12];  // Raw byte array access
};
// Total: 12 bytes (packed, no padding)

// RX Data Buffer with union for easy byte access
// Protocol: CMD1 | POS1(4 bytes) | STAT1 | I1(4 bytes) | CMD2 | POS2(4 bytes) | STAT2 | I2(4 bytes)
union RS485_rxdata_buf {
  struct __attribute__((packed)) {
    uint8_t CMD1;   // Byte 0: RX CMD/Version for steering motor
    int32_t POS1;   // Bytes 1-4: Position for steering motor
    uint8_t STAT1;  // Byte 5: Status for steering motor
    int32_t I1;     // Bytes 6-9: Current of steering motor
    uint8_t CMD2;   // Byte 10: RX CMD/Version for wheel hub motor
    int32_t POS2;   // Bytes 11-14: Position for wheel hub motor
    uint8_t STAT2;  // Byte 15: Status for wheel hub motor
    int32_t I2;     // Bytes 16-19: Current of wheel hub motor
  };
  uint8_t bytes[20];  // Raw byte array access
};
// Total: 20 bytes (packed, no padding)
// FPGA driver adds: Header (5 bytes) + Payload (20 bytes) + Checksum (2 bytes) = 27 bytes total
// Byte		  |   0~4  |  5   | 6~9  |   10  | 11~14 |  15  | 16~19 |   20  |  21~24  |   25|   26
// Function	| Header | CMD1 | POS1 | STAT1 |   I1  | CMD2 | POS2  | STAT2 |    I2   |Checksum1|Checksum2|

// Position data that can be either int32 or float32
union MotorPosition {
  int32_t as_int;
  float as_float;

  MotorPosition() : as_int(0) {}
  MotorPosition(int32_t val) : as_int(val) {}
  MotorPosition(float val) : as_float(val) {}
};

struct Motor_RS485 {
  int id_;
  bool is_steering_;  // true for steering (uses float), false for wheel (uses int32)

  // Command values (desired, sent to motor)
  MotorPosition pos_des_;  // float for steering, int32 for wheel
  double vel_des_;
  double trq_des_;
  MotorMode mode_des_;

  // Feedback values (actual, received from motor)
  MotorPosition pos_act_;  // float for steering, int32 for wheel
  double vel_act_;
  double trq_act_;
  MotorMode mode_act_;
};

class LimbModule {
 public:
  LimbModule(std::string _label, YAML::Node _config, NiFpga_Status _status, NiFpga_Session _fpga_session,
             int rs485_port);
  LimbModule() {}

  std::string label_;
  Motor_RS485 steering_motor;
  Motor_RS485 wheel_motor;

  // RS485 communication methods
  void send_motor_commands();
  void receive_motor_feedback();
  void update_motors();

  // Motor control methods
  void set_steering_position(double position);
  void set_steering_velocity(double velocity);
  void set_steering_torque(double torque);
  void set_wheel_velocity(double velocity);
  void set_wheel_torque(double torque);

  // Getters
  float get_steering_position() const { return steering_motor.pos_act_.as_float; }
  double get_steering_velocity() const { return steering_motor.vel_act_; }
  double get_steering_torque() const { return steering_motor.trq_act_; }
  int32_t get_wheel_position() const { return wheel_motor.pos_act_.as_int; }
  double get_wheel_velocity() const { return wheel_motor.vel_act_; }
  double get_wheel_torque() const { return wheel_motor.trq_act_; }

  // Debug getters for mode change management
  MotorMode get_prev_mode_des_steering() const { return prev_mode_des_steering_; }
  MotorMode get_prev_mode_des_wheel() const { return prev_mode_des_wheel_; }
  bool get_mode_change_sent_steering() const { return mode_change_sent_steering_; }
  bool get_mode_change_sent_wheel() const { return mode_change_sent_wheel_; }

  bool is_communication_ok() const { return !RS485_module_timedout; }

  void Helloworld() { std::cout << "Hello from LimbModule!" << std::endl; }

  // Debug access (public for console display)
  ModuleIO_RS485 io_;
  int rs485_port_;
  RS485_txdata_buf txdata_buffer_;
  RS485_rxdata_buf rxdata_buffer_;
  uint8_t raw_rx_buffer_[32];  // Raw RX buffer from FPGA (including header/checksum)
  bool RS485_tx_timedout;
  bool RS485_rx_timedout;
  bool RS485_module_timedout;

 private:
  static constexpr double MAX_STEERING_POSITION = 5.0;  // radians

  NiFpga_Status status_;
  NiFpga_Session fpga_session_;

  // Timeout management
  int RS485_timeout_us_;
  int32_t last_tx_count_;  // Per-instance counter for timeout detection
  int32_t last_rx_count_;  // Per-instance counter for timeout detection

  // Mode change management
  MotorMode prev_mode_des_steering_;  // Track previous desired mode to detect changes
  MotorMode prev_mode_des_wheel_;
  bool mode_change_sent_steering_;  // Flag indicating mode change command was sent
  bool mode_change_sent_wheel_;

  // Calibration state tracking
  bool cal_sent_steering_;  // Track if calibration command has been sent for steering motor

  // Helper methods
  void load_config();
  void RS485_timeoutCheck();
  bool pack_tx_buffer();
  void unpack_rx_buffer();
  uint8_t calculate_checksum(const uint8_t* data, size_t length);
  bool verify_checksum(const uint8_t* data, size_t length);
};

#endif

// below is the RS485 communication protocol reference from the motor controller code
/*	Packet TX Information (from the motor controllers point of view)

Byte		  |   0~4  |  5   | 6~9  |   10  | 11~14 |  15  | 16~19 |   20  |  21~24  |   25|   26
Function	| Header | CMD1 | POS1 | STAT1 |   I1  | CMD2 | POS2  | STAT2 |    I2   |Checksum1|Checksum2|

        Byte0~4		: Header, 0xFF*5
        ------------------------------------------------------------------
        Byte5		: CMD1, RX CMD/Version for steering motor, firmware version(bit4~6) + CMD(0~3)
        Byte6~9		: POS1, position for steering motor
        Byte10		: STAT1, status for steering motor, hall-status(bit4~5) + system state(0~3)
        Byte11~14	: I1, current of steering motor
        ------------------------------------------------------------------
        Byte15		: CMD2, RX CMD/Version for wheel hub motor, firmware version(bit4~6) + CMD(0~3)
        Byte16~19	: POS2, position for wheel hub motor
        Byte20		: STAT2, status for wheel hub motor
        Byte21~24	: I2, current of wheel hub motor
        ------------------------------------------------------------------
        Byte25		: Checksum1(CKS1), calculate by following steps
                                  Step1 : XOR(^) all packet data, exclusive Header, CKS1&2
                                  Step2 : Make result of Step1 & 0xFE
        Byte26		: Checksum2(CKS1), (~CKS1) & 0xFE
*/
/*	Packet RX Information (from the motor controllers point of view)

        Byte		|   0~4  |    5    |    6    |  7~10   |   11    |   12    |  13~16  |   17    |   18    |
        Function	| Header |   CMD1  | SUBCMD1 |  Data1  |   CMD2  | SUBCMD2 |  Data2  |Checksum1|Checksum2|

        Byte0~4		: Header, 0xFF*5
        ------------------------------------------------------------------
        Byte5		: CMD1, commnad for steering motor
        Byte6		: SUBCMD1, sub commnad for steering motor
        Byte7~10	: Data1, data for steering motor
        ------------------------------------------------------------------
        Byte11		: CMD2, commnad for wheel hub motor
        Byte12		: SUBCMD2, sub commnad for wheel hub motor
        Byte12~16	: Data2, data for wheel hub motor
        ------------------------------------------------------------------
        Byte17		: Checksum1(CKS1), calculate by following steps
                                  Step1 : XOR(^) all packet data, exclusive Header, CKS1&2
                                  Step2 : Make result of Step1 & 0xFE
        Byte18		: Checksum2(CKS1), (~CKS1) & 0xFE
*/
