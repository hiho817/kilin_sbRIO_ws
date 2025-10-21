#ifndef __FPGAHANDLER_H
#define __FPGAHANDLER_H

#include <curses.h>
#include <dlfcn.h>
#include <ncurses.h>
#include <signal.h>
#include <unistd.h>

#include <bitset>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

#include "NiFpga.h"
#include "NiFpga_FPGA_RS485_v1_2.h"
#include "can_packet.h"
#include "color.hpp"
#include "msg.hpp"
#undef OK

class ModuleIO_CAN {
 public:
  ModuleIO_CAN(NiFpga_Status status_, NiFpga_Session fpga_session_, std::string CAN_port_,
               std::vector<Motor>* motors_list);

  ModuleIO_CAN() {};

  NiFpga_Status get_fpga_status() { return status_; };

  // read write function
  void set_ni_CAN_transmit(NiFpga_Bool value);
  void set_ni_CAN_id(uint32_t id1, uint32_t id2);
  void set_ni_port_select(const NiFpga_Bool* array);
  void set_ni_CAN_id_fc(uint32_t id1_fc, uint32_t id2_fc);

  void get_ni_CAN_id_fc(uint32_t* fc1, uint32_t* fc2);
  void get_ni_rx_data(uint8_t* rx_arr1, uint8_t* rx_arr2);

  NiFpga_Bool get_ni_CAN_complete();
  NiFpga_Bool get_ni_CAN_success();
  int16_t get_ni_CAN_complete_counter();

  NiFpga_Bool get_ni_tx_timeout();
  NiFpga_Bool get_ni_rx_timeout();

  void CAN_setup(int timeout_us);
  void CAN_set_mode(Mode mode);
  void CAN_send_command(CAN_txdata txdata_id1, CAN_txdata txdata_id2);
  void CAN_recieve_feedback(CAN_rxdata* rxdata_id1, CAN_rxdata* rxdata_id2);

  // motor bias getters and setters
  double get_motor_F_bias() { return motor_F_bias_; }
  double get_motor_H_bias() { return motor_H_bias_; }
  void set_motor_F_bias(double bias) { motor_F_bias_ = bias; }
  void set_motor_H_bias(double bias) { motor_H_bias_ = bias; }

 private:
  int CAN_timeout_us_;

  NiFpga_FPGA_RS485_v1_2_ControlU32 r_CAN_id1_;
  NiFpga_FPGA_RS485_v1_2_ControlU32 r_CAN_id2_;
  NiFpga_FPGA_RS485_v1_2_ControlU32 r_CAN_id1_FC_;
  NiFpga_FPGA_RS485_v1_2_ControlU32 r_CAN_id2_FC_;

  NiFpga_FPGA_RS485_v1_2_ControlArrayBool r_port_select_;
  NiFpga_FPGA_RS485_v1_2_ControlArrayBoolSize r_port_select_size_;

  // tx buffer
  NiFpga_FPGA_RS485_v1_2_ControlArrayU8 r_tx_buf_id1_;
  NiFpga_FPGA_RS485_v1_2_ControlArrayU8 r_tx_buf_id2_;
  NiFpga_FPGA_RS485_v1_2_ControlArrayU8Size r_tx_buf_size_;

  // rx buffer
  NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8 r_rx_buf_id1_;
  NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8 r_rx_buf_id2_;
  NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8Size r_rx_buf_size_;

  NiFpga_FPGA_RS485_v1_2_ControlBool r_CAN_transmit_;
  NiFpga_FPGA_RS485_v1_2_IndicatorBool r_CAN_complete_;
  NiFpga_FPGA_RS485_v1_2_IndicatorBool r_CAN_success_;
  NiFpga_FPGA_RS485_v1_2_IndicatorI16 r_CAN_complete_counter_;

  NiFpga_FPGA_RS485_v1_2_IndicatorBool r_tx_timeout_;
  NiFpga_FPGA_RS485_v1_2_IndicatorBool r_rx_timeout_;
  NiFpga_FPGA_RS485_v1_2_ControlU32 r_timeout_us_;

  double motor_F_bias_;
  double motor_H_bias_;

  void CAN_encode_(uint8_t (&txmsg)[8], CAN_txdata txdata);
  void CAN_decode_(uint8_t (&rxmsg)[8], CAN_rxdata* rxdata);
  // data conversion for CAN-bus
  int float_to_uint_(float x, float x_min, float x_max, int bits);
  float uint_to_float_(int x_int, float x_min, float x_max, int bits);
  void set_ni_tx_data_(const uint8_t* tx_arr1, const uint8_t* tx_arr2);
  void set_ni_timeout_us_(uint32_t value);
  NiFpga_Status set_fpga_status(const NiFpga_Status newStatus) {
    return NiFpga_MergeStatus(&status_, newStatus);
  };

  NiFpga_Status status_;
  NiFpga_Session fpga_session_;
  std::vector<Motor>* motors_list_;
};

class FpgaHandler {
 public:
  FpgaHandler();
  ~FpgaHandler();

  NiFpga_Session session;
  NiFpga_IrqContext irqContext;

  NiFpga_Status get_fpga_status() { return status_; };
  NiFpga_Status set_fpga_status(const NiFpga_Status newStatus) {
    return NiFpga_MergeStatus(&status_, newStatus);
  };

  void set_ni_irq_period(int main_loop_period, int can_loop_period);
  void set_ni_pwrb(std::vector<bool>* powerboard_state_);
  void get_ni_pwrb_to_buf();

  double powerboard_Ifactor[12];
  double powerboard_Ioffset[12];
  double powerboard_Vfactor[12];
  double powerboard_Voffset[12];

  double pwrb_I_buf[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  double pwrb_V_buf[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

 private:
  NiFpga_Status status_;
  // Fpga interrupt request

  // powerboard
  NiFpga_FPGA_RS485_v1_2_ControlBool w_pb_digital_;
  NiFpga_FPGA_RS485_v1_2_ControlBool w_pb_signal_;
  NiFpga_FPGA_RS485_v1_2_ControlBool w_pb_power_;

  NiFpga_FPGA_RS485_v1_2_IndicatorArrayU16 r_powerboard_data_;
  NiFpga_FPGA_RS485_v1_2_IndicatorArrayU16Size size_powerboard_data_;
};

#endif
