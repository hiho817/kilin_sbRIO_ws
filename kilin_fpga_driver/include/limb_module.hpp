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

// TX Data Buffer with union for easy byte access
union RS485_txdata_buf {
  struct __attribute__((packed)) {
    uint8_t CMD1;       // Byte 0
    uint8_t SUBCMD1;    // Byte 1
    uint32_t Data1;     // Bytes 2-5
    uint8_t CMD2;       // Byte 6
    uint8_t SUBCMD2;    // Byte 7
    uint32_t Data2;     // Bytes 8-11
  };
  uint8_t bytes[12];    // Raw byte array access
};
// Total: 12 bytes (packed, no padding)
// FPGA driver adds: Header (5 bytes) + Payload (12 bytes) + Checksum (2 bytes) = 19 bytes total

// RX Data Buffer with union for easy byte access
union RS485_rxdata_buf {
  struct __attribute__((packed)) {
    uint8_t CMD1;       // Byte 0
    uint8_t SUBCMD1;    // Byte 1
    uint32_t Data1;     // Bytes 2-5
    uint8_t CMD2;       // Byte 6
    uint8_t SUBCMD2;    // Byte 7
    uint32_t Data2;     // Bytes 8-11
  };
  uint8_t bytes[12];    // Raw byte array access
};
// Total: 12 bytes (packed, no padding)
// Byte		  |   0~4  |  5   | 6~9  |   10  | 11~14 |  15  | 16~19 |   20  |  21~24  |   25|   26    
// Function	| Header | CMD1 | POS1 | STAT1 |   I1  | CMD2 | POS2  | STAT2 |    I2   |Checksum1|Checksum2|

struct Motor_RS485 {
  int id_;
  double position_;
  double velocity_;
  double torque_;
  double kp_;
  double ki_;
  double kd_;
  double kt_;
  MotorMode mode_;
};

class LimbModule {
 public:
  LimbModule(std::string _label, YAML::Node _config, NiFpga_Status _status, NiFpga_Session _fpga_session, int rs485_port);
  LimbModule() {}

  std::string label_;
  Motor_RS485 steering_motor;
  Motor_RS485 wheel_motor;

  // RS485 communication methods
  void send_motor_commands();
  void receive_motor_feedback();
  void update_motors();
  
  // Motor control methods
  void set_steering_position(double position, double kp, double kd);
  void set_steering_velocity(double velocity, double kp, double kd);
  void set_steering_torque(double torque);
  void set_wheel_velocity(double velocity, double kp, double kd);
  void set_wheel_torque(double torque);
  
  // Getters
  double get_steering_position() const { return steering_motor.position_; }
  double get_steering_velocity() const { return steering_motor.velocity_; }
  double get_steering_torque() const { return steering_motor.torque_; }
  double get_wheel_position() const { return wheel_motor.position_; }
  double get_wheel_velocity() const { return wheel_motor.velocity_; }
  double get_wheel_torque() const { return wheel_motor.torque_; }
  
  bool is_communication_ok() const { return !RS485_module_timedout; }
  
  void Helloworld() { std::cout << "Hello from LimbModule!" << std::endl; }

  // Debug access (public for console display)
  ModuleIO_RS485 io_;
  int rs485_port_;
  RS485_txdata_buf txdata_buffer_;
  RS485_rxdata_buf rxdata_buffer_;
  bool RS485_tx_timedout_[2];  // [0] for steering, [1] for wheel
  bool RS485_rx_timedout_[2];
  bool RS485_mtr_timedout[2];
  bool RS485_module_timedout;

 private:
  NiFpga_Status status_;
  NiFpga_Session fpga_session_;
  
  // Timeout management
  int RS485_timeout_us_;
  
  // Helper methods
  void load_config();
  void RS485_timeoutCheck();
  void pack_tx_buffer();
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
