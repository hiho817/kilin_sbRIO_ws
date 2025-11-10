#include <fpga_handler.hpp>

ModuleIO_CAN::ModuleIO_CAN(NiFpga_Status status, NiFpga_Session fpga_session, std::string CAN_port_)
    : status_(status), fpga_session_(fpga_session) {
  if (CAN_port_ == "MOD1CAN0") {
    r_CAN_id1_ = NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN0ID1;
    r_CAN_id2_ = NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN0ID2;

    r_CAN_id1_FC_ = NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN0ID1FC;
    r_CAN_id2_FC_ = NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN0ID2FC;

    r_port_select_ = NiFpga_FPGA_RS485_v1_2_ControlArrayBool_Mod1CAN0Select;
    r_port_select_size_ = NiFpga_FPGA_RS485_v1_2_ControlArrayBoolSize_Mod1CAN0Select;

    r_tx_buf_id1_ = NiFpga_FPGA_RS485_v1_2_ControlArrayU8_Mod1CAN0ID1TX;
    r_tx_buf_id2_ = NiFpga_FPGA_RS485_v1_2_ControlArrayU8_Mod1CAN0ID2TX;
    r_tx_buf_size_ = NiFpga_FPGA_RS485_v1_2_ControlArrayU8Size_Mod1CAN0ID1TX;

    r_rx_buf_id1_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_Mod1CAN0ID1RX;
    r_rx_buf_id2_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_Mod1CAN0ID2RX;
    r_rx_buf_size_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8Size_Mod1CAN0ID1RX;

    r_CAN_transmit_ = NiFpga_FPGA_RS485_v1_2_ControlBool_MOD1CAN0Transmit;
    r_CAN_complete_ = NiFpga_FPGA_RS485_v1_2_IndicatorBool_Mod1CAN0Complete;
    r_CAN_success_ = NiFpga_FPGA_RS485_v1_2_IndicatorBool_Mod1CAN0success;
    r_CAN_complete_counter_ = NiFpga_FPGA_RS485_v1_2_IndicatorI16_Mod1CAN0CompleteCounter;

    r_tx_timeout_ = NiFpga_FPGA_RS485_v1_2_IndicatorBool_Mod1CAN0TXTimeout;
    r_rx_timeout_ = NiFpga_FPGA_RS485_v1_2_IndicatorBool_Mod1CAN0RXTimeout;

    r_timeout_us_ = NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN0RXTimeoutus;
  } else if (CAN_port_ == "MOD1CAN1") {
    r_CAN_id1_ = NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN1ID1;
    r_CAN_id2_ = NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN1ID2;

    r_CAN_id1_FC_ = NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN1ID1FC;
    r_CAN_id2_FC_ = NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN1ID2FC;

    r_port_select_ = NiFpga_FPGA_RS485_v1_2_ControlArrayBool_Mod1CAN1Select;
    r_port_select_size_ = NiFpga_FPGA_RS485_v1_2_ControlArrayBoolSize_Mod1CAN1Select;

    r_tx_buf_id1_ = NiFpga_FPGA_RS485_v1_2_ControlArrayU8_Mod1CAN1ID1TX;
    r_tx_buf_id2_ = NiFpga_FPGA_RS485_v1_2_ControlArrayU8_Mod1CAN1ID2TX;
    r_tx_buf_size_ = NiFpga_FPGA_RS485_v1_2_ControlArrayU8Size_Mod1CAN1ID1TX;

    r_rx_buf_id1_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_Mod1CAN1ID1RX;
    r_rx_buf_id2_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_Mod1CAN1ID2RX;
    r_rx_buf_size_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8Size_Mod1CAN1ID1RX;

    r_CAN_transmit_ = NiFpga_FPGA_RS485_v1_2_ControlBool_MOD1CAN1Transmit;
    r_CAN_complete_ = NiFpga_FPGA_RS485_v1_2_IndicatorBool_Mod1CAN1Complete;
    r_CAN_success_ = NiFpga_FPGA_RS485_v1_2_IndicatorBool_Mod1CAN1success;
    r_CAN_complete_counter_ = NiFpga_FPGA_RS485_v1_2_IndicatorI16_Mod1CAN1CompleteCounter;

    r_tx_timeout_ = NiFpga_FPGA_RS485_v1_2_IndicatorBool_Mod1CAN1TXTimeout;
    r_rx_timeout_ = NiFpga_FPGA_RS485_v1_2_IndicatorBool_Mod1CAN1RXTimeout;

    r_timeout_us_ = NiFpga_FPGA_RS485_v1_2_ControlU32_Mod1CAN1RXTimeoutus;
  } else {
    std::cout << "[ERROR] CAN_PORT CONFIG ERROR !" << std::endl;
  }
}

void ModuleIO_CAN::set_ni_CAN_id(uint32_t id1, uint32_t id2) {
  set_fpga_status(NiFpga_WriteU32(fpga_session_, r_CAN_id1_, id1));
  set_fpga_status(NiFpga_WriteU32(fpga_session_, r_CAN_id2_, id2));
}

void ModuleIO_CAN::set_ni_CAN_id1(uint32_t id1) {
  set_fpga_status(NiFpga_WriteU32(fpga_session_, r_CAN_id1_, id1));
}

void ModuleIO_CAN::set_ni_CAN_id2(uint32_t id2) {
  set_fpga_status(NiFpga_WriteU32(fpga_session_, r_CAN_id2_, id2));
}

void ModuleIO_CAN::set_ni_CAN_id_fc(uint32_t id1_fc, uint32_t id2_fc) {
  set_fpga_status(NiFpga_WriteU32(fpga_session_, r_CAN_id1_FC_, id1_fc));
  set_fpga_status(NiFpga_WriteU32(fpga_session_, r_CAN_id2_FC_, id2_fc));
}

void ModuleIO_CAN::get_ni_CAN_id_fc(uint32_t* fc1, uint32_t* fc2) {
  set_fpga_status(NiFpga_ReadU32(fpga_session_, r_CAN_id1_FC_, fc1));
  set_fpga_status(NiFpga_ReadU32(fpga_session_, r_CAN_id2_FC_, fc2));
}

void ModuleIO_CAN::set_ni_port_select(const NiFpga_Bool* array) {
  set_fpga_status(NiFpga_WriteArrayBool(fpga_session_, r_port_select_, array, r_port_select_size_));
}

void ModuleIO_CAN::set_ni_tx_data(const uint8_t* tx_arr1, const uint8_t* tx_arr2) {
  set_fpga_status(NiFpga_WriteArrayU8(fpga_session_, r_tx_buf_id1_, tx_arr1, r_tx_buf_size_));
  set_fpga_status(NiFpga_WriteArrayU8(fpga_session_, r_tx_buf_id2_, tx_arr2, r_tx_buf_size_));
}

void ModuleIO_CAN::set_ni_tx_data_motor1(const uint8_t* tx_arr) {
  // Write directly to FPGA register for motor 1
  set_fpga_status(NiFpga_WriteArrayU8(fpga_session_, r_tx_buf_id1_, tx_arr, r_tx_buf_size_));
}

void ModuleIO_CAN::set_ni_tx_data_motor2(const uint8_t* tx_arr) {
  // Write directly to FPGA register for motor 2
  set_fpga_status(NiFpga_WriteArrayU8(fpga_session_, r_tx_buf_id2_, tx_arr, r_tx_buf_size_));
}

void ModuleIO_CAN::set_ni_CAN_transmit(NiFpga_Bool value) {
  set_fpga_status(NiFpga_WriteBool(fpga_session_, r_CAN_transmit_, value));
}

void ModuleIO_CAN::set_ni_timeout_us(uint32_t value) {
  set_fpga_status(NiFpga_WriteU32(fpga_session_, r_timeout_us_, value));
}

void ModuleIO_CAN::get_ni_rx_data(uint8_t* rx_arr1, uint8_t* rx_arr2) {
  set_fpga_status(NiFpga_ReadArrayU8(fpga_session_, r_rx_buf_id1_, rx_arr1, r_rx_buf_size_));
  set_fpga_status(NiFpga_ReadArrayU8(fpga_session_, r_rx_buf_id2_, rx_arr2, r_rx_buf_size_));
}

void ModuleIO_CAN::get_ni_tx_data(uint8_t* tx_arr1, uint8_t* tx_arr2) {
  set_fpga_status(NiFpga_ReadArrayU8(fpga_session_, r_tx_buf_id1_, tx_arr1, r_tx_buf_size_));
  set_fpga_status(NiFpga_ReadArrayU8(fpga_session_, r_tx_buf_id2_, tx_arr2, r_tx_buf_size_));
}

NiFpga_Bool ModuleIO_CAN::get_ni_CAN_complete() {
  NiFpga_Bool complete = 0;
  set_fpga_status(NiFpga_ReadBool(fpga_session_, r_CAN_complete_, &complete));
  return complete;
}

NiFpga_Bool ModuleIO_CAN::get_ni_CAN_success() {
  NiFpga_Bool success = 0;
  set_fpga_status(NiFpga_ReadBool(fpga_session_, r_CAN_success_, &success));
  return success;
}

int16_t ModuleIO_CAN::get_ni_CAN_complete_counter() {
  int16_t count = 0;
  set_fpga_status(NiFpga_ReadI16(fpga_session_, r_CAN_complete_counter_, &count));
  return count;
}

NiFpga_Bool ModuleIO_CAN::get_ni_tx_timeout() {
  NiFpga_Bool id1_timeout = 0;
  set_fpga_status(NiFpga_ReadBool(fpga_session_, r_tx_timeout_, &id1_timeout));
  return id1_timeout;
}

NiFpga_Bool ModuleIO_CAN::get_ni_rx_timeout() {
  NiFpga_Bool id1_timeout = 0;
  set_fpga_status(NiFpga_ReadBool(fpga_session_, r_rx_timeout_, &id1_timeout));
  return id1_timeout;
}

// ============================================================================
// PwrbIO Implementation
// ============================================================================

void PwrbIO::set_ni_pwrb(std::vector<bool>* powerboard_state_) {
  set_fpga_status(NiFpga_WriteBool(fpga_session_, w_pb_digital_, powerboard_state_->at(0)));
  set_fpga_status(NiFpga_WriteBool(fpga_session_, w_pb_signal_, powerboard_state_->at(1)));
  set_fpga_status(NiFpga_WriteBool(fpga_session_, w_pb_power_, powerboard_state_->at(2)));
}

void PwrbIO::get_ni_pwrb_to_buf() {
  uint16_t rx_arr[24];
  set_fpga_status(NiFpga_ReadArrayU16(fpga_session_, NiFpga_FPGA_RS485_v1_2_IndicatorArrayU16_Data, rx_arr,
                                      NiFpga_FPGA_RS485_v1_2_IndicatorArrayU16Size_Data));
  for (int i = 0; i < 24; i++) {
    int i_half = i / 2;
    if (i % 2 == 1)  // is odd
      set_v_buf_(i_half, rx_arr[i] * pwrb_cal_params_.V.factor[i_half] + pwrb_cal_params_.V.factor[i_half]);
    else // is even
      set_i_buf_(i_half, rx_arr[i] * pwrb_cal_params_.I.factor[i_half] + pwrb_cal_params_.I.offset[i_half]);
  }
}

PwrbIO::PwrbIO(NiFpga_Status status, NiFpga_Session fpga_session)
    : status_(status),
      fpga_session_(fpga_session),
      w_pb_digital_(NiFpga_FPGA_RS485_v1_2_ControlBool_Digital),
      w_pb_signal_(NiFpga_FPGA_RS485_v1_2_ControlBool_Signal),
      w_pb_power_(NiFpga_FPGA_RS485_v1_2_ControlBool_Power),
      r_powerboard_data_(NiFpga_FPGA_RS485_v1_2_IndicatorArrayU16_Data),
      size_powerboard_data_(NiFpga_FPGA_RS485_v1_2_IndicatorArrayU16Size_Data) {}

void PwrbIO::set_pwrb_cal_params_from_yml(const YAML::Node& factors_node) {
  std::cout << "Loading PowerBoard Scaling Factors..." << std::endl;
  size_t idx = 0;
  for (const auto& f : factors_node) {
    if (idx >= pwrb_cal_params_.V.factor.size()) {
      std::cerr << "Warning: More calibration entries in YAML than expected. Ignoring extra." << std::endl;
      break;
    }

    pwrb_cal_params_.I.factor[idx] = f["Current_Factor"].as<double>();
    pwrb_cal_params_.I.offset[idx] = f["Current_Offset"].as<double>();
    pwrb_cal_params_.V.factor[idx] = f["Voltage_Factor"].as<double>();
    pwrb_cal_params_.V.offset[idx] = f["Voltage_Offset"].as<double>();

    idx++;
  }
}

double PwrbIO::get_v_factor(size_t index) const {
  if (index >= pwrb_cal_params_.V.factor.size()) {
    throw std::out_of_range("Index out of range for voltage factor.");
  }
  return pwrb_cal_params_.V.factor[index];
}

double PwrbIO::get_v_offset(size_t index) const {
  if (index >= pwrb_cal_params_.V.offset.size()) {
    throw std::out_of_range("Index out of range for voltage offset.");
  }
  return pwrb_cal_params_.V.offset[index];
}

double PwrbIO::get_i_factor(size_t index) const {
  if (index >= pwrb_cal_params_.I.factor.size()) {
    throw std::out_of_range("Index out of range for current factor.");
  }
  return pwrb_cal_params_.I.factor[index];
}

double PwrbIO::get_i_offset(size_t index) const {
  if (index >= pwrb_cal_params_.I.offset.size()) {
    throw std::out_of_range("Index out of range for current offset.");
  }
  return pwrb_cal_params_.I.offset[index];
}

// Setter for a single voltage value
void PwrbIO::set_v_buf_(size_t index, double value) {
  // Bounds checking makes this much safer!
  if (index >= buffers_.voltage_buffer.size()) {
    throw std::out_of_range("Voltage buffer index is out of bounds.");
  }
  buffers_.voltage_buffer[index] = value;
}

// Getter for a single voltage value
double PwrbIO::get_v_buf(size_t index) const {
  if (index >= buffers_.voltage_buffer.size()) {
    throw std::out_of_range("Voltage buffer index is out of bounds.");
  }
  return buffers_.voltage_buffer[index];
}

// Setter for a single current value
void PwrbIO::set_i_buf_(size_t index, double value) {
  if (index >= buffers_.current_buffer.size()) {
    throw std::out_of_range("Current buffer index is out of bounds.");
  }
  buffers_.current_buffer[index] = value;
}

// Getter for a single current value
double PwrbIO::get_i_buf(size_t index) const {
  if (index >= buffers_.current_buffer.size()) {
    throw std::out_of_range("Current buffer index is out of bounds.");
  }
  return buffers_.current_buffer[index];
}

// ============================================================================
// ModuleIO_RS485 Implementation
// ============================================================================

ModuleIO_RS485::ModuleIO_RS485(NiFpga_Status status, NiFpga_Session fpga_session, int rs485_port)
    : status_(status),
      fpga_session_(fpga_session),
      rs485_port_(rs485_port) {
  init_registers_();
}

void ModuleIO_RS485::init_registers_() {
  // Initialize registers based on the RS485 port (1-4)
  switch (rs485_port_) {
    case 1:
      r_RS485_transmit_ = NiFpga_FPGA_RS485_v1_2_ControlBool_RS485_Transmit1;
      r_tx_data_ = NiFpga_FPGA_RS485_v1_2_ControlArrayU8_RS485_TX_Data1;
      r_tx_data_size_ = NiFpga_FPGA_RS485_v1_2_ControlArrayU8Size_RS485_TX_Data1;
      r_rx_data_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_RS485_RX_Data1;
      r_rx_data_size_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8Size_RS485_RX_Data1;
      r_rx_buf_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_RS485_RX_Buf1;
      r_rx_buf_size_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8Size_RS485_RX_Buf1;
      r_checksum_ok_ = NiFpga_FPGA_RS485_v1_2_IndicatorBool_RS485_CKS_OK1;
      r_rx_finish_ = NiFpga_FPGA_RS485_v1_2_IndicatorBool_RS485_RX_finish1;
      r_rx_count_ = NiFpga_FPGA_RS485_v1_2_IndicatorI32_RS485_RX_count1;
      r_tx_count_ = NiFpga_FPGA_RS485_v1_2_IndicatorI32_RS485_TX_count1;
      break;
    case 2:
      r_RS485_transmit_ = NiFpga_FPGA_RS485_v1_2_ControlBool_RS485_Transmit2;
      r_tx_data_ = NiFpga_FPGA_RS485_v1_2_ControlArrayU8_RS485_TX_Data2;
      r_tx_data_size_ = NiFpga_FPGA_RS485_v1_2_ControlArrayU8Size_RS485_TX_Data2;
      r_rx_data_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_RS485_RX_Data2;
      r_rx_data_size_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8Size_RS485_RX_Data2;
      r_rx_buf_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_RS485_RX_Buf2;
      r_rx_buf_size_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8Size_RS485_RX_Buf2;
      r_checksum_ok_ = NiFpga_FPGA_RS485_v1_2_IndicatorBool_RS485_CKS_OK2;
      r_rx_finish_ = NiFpga_FPGA_RS485_v1_2_IndicatorBool_RS485_RX_finish2;
      r_rx_count_ = NiFpga_FPGA_RS485_v1_2_IndicatorI32_RS485_RX_count2;
      r_tx_count_ = NiFpga_FPGA_RS485_v1_2_IndicatorI32_RS485_TX_count2;
      break;
    case 3:
      r_RS485_transmit_ = NiFpga_FPGA_RS485_v1_2_ControlBool_RS485_Transmit3;
      r_tx_data_ = NiFpga_FPGA_RS485_v1_2_ControlArrayU8_RS485_TX_Data3;
      r_tx_data_size_ = NiFpga_FPGA_RS485_v1_2_ControlArrayU8Size_RS485_TX_Data3;
      r_rx_data_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_RS485_RX_Data3;
      r_rx_data_size_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8Size_RS485_RX_Data3;
      r_rx_buf_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_RS485_RX_Buf3;
      r_rx_buf_size_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8Size_RS485_RX_Buf3;
      r_checksum_ok_ = NiFpga_FPGA_RS485_v1_2_IndicatorBool_RS485_CKS_OK3;
      r_rx_finish_ = NiFpga_FPGA_RS485_v1_2_IndicatorBool_RS485_RX_finish3;
      r_rx_count_ = NiFpga_FPGA_RS485_v1_2_IndicatorI32_RS485_RX_count3;
      r_tx_count_ = NiFpga_FPGA_RS485_v1_2_IndicatorI32_RS485_TX_count3;
      break;
    case 4:
      r_RS485_transmit_ = NiFpga_FPGA_RS485_v1_2_ControlBool_RS485_Transmit4;
      r_tx_data_ = NiFpga_FPGA_RS485_v1_2_ControlArrayU8_RS485_TX_Data4;
      r_tx_data_size_ = NiFpga_FPGA_RS485_v1_2_ControlArrayU8Size_RS485_TX_Data4;
      r_rx_data_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_RS485_RX_Data4;
      r_rx_data_size_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8Size_RS485_RX_Data4;
      r_rx_buf_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8_RS485_RX_Buf4;
      r_rx_buf_size_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU8Size_RS485_RX_Buf4;
      r_checksum_ok_ = NiFpga_FPGA_RS485_v1_2_IndicatorBool_RS485_CKS_OK4;
      r_rx_finish_ = NiFpga_FPGA_RS485_v1_2_IndicatorBool_RS485_RX_finish4;
      r_rx_count_ = NiFpga_FPGA_RS485_v1_2_IndicatorI32_RS485_RX_count4;
      r_tx_count_ = NiFpga_FPGA_RS485_v1_2_IndicatorI32_RS485_TX_count4;
      break;
    default:
      error_message("[RS485] Invalid RS485 port number. Must be 1-4.");
      break;
  }
}

void ModuleIO_RS485::set_ni_RS485_transmit(NiFpga_Bool value) {
  set_fpga_status(NiFpga_WriteBool(fpga_session_, r_RS485_transmit_, value));
}

void ModuleIO_RS485::set_ni_tx_data(const uint8_t* tx_data) {
  // Write the TX data to FPGA using the internally stored buffer size
  // Note: Caller must ensure tx_data buffer is exactly r_tx_data_size_ bytes (12 bytes for RS485)
  set_fpga_status(NiFpga_WriteArrayU8(fpga_session_, r_tx_data_, tx_data, r_tx_data_size_));
}

void ModuleIO_RS485::get_ni_rx_data(uint8_t* rx_data) {
  set_fpga_status(NiFpga_ReadArrayU8(fpga_session_, r_rx_data_, rx_data, r_rx_data_size_));
}

void ModuleIO_RS485::get_ni_rx_buf(uint8_t* rx_buf) {
  set_fpga_status(NiFpga_ReadArrayU8(fpga_session_, r_rx_buf_, rx_buf, r_rx_buf_size_));
}

NiFpga_Bool ModuleIO_RS485::get_ni_checksum_ok() {
  NiFpga_Bool value;
  set_fpga_status(NiFpga_ReadBool(fpga_session_, r_checksum_ok_, &value));
  return value;
}

NiFpga_Bool ModuleIO_RS485::get_ni_rx_finish() {
  NiFpga_Bool value;
  set_fpga_status(NiFpga_ReadBool(fpga_session_, r_rx_finish_, &value));
  return value;
}

int32_t ModuleIO_RS485::get_ni_rx_count() {
  int32_t value;
  set_fpga_status(NiFpga_ReadI32(fpga_session_, r_rx_count_, &value));
  return value;
}

int32_t ModuleIO_RS485::get_ni_tx_count() {
  int32_t value;
  set_fpga_status(NiFpga_ReadI32(fpga_session_, r_tx_count_, &value));
  return value;
}

// ============================================================================
// FpgaHandler Implementation
// ============================================================================

FpgaHandler::FpgaHandler() {
  // init the NiFpga system
  status_ = NiFpga_Initialize();
  if (NiFpga_IsError(status_)) {
    error_message("[FPGA Handler] Failed to initialize NI-FPGA. Error code: " + std::to_string(status_));
    error_message("[FPGA Handler] Please ensure NI-RIO drivers are installed and sbRIO is powered on.");
    return;
  }
  important_message("[FPGA Handler] Fpga Initialized");

  // init the session variable
  std::cout << "[FPGA Handler] Opening FPGA session with bitfile: " << NiFpga_FPGA_RS485_v1_2_Bitfile << std::endl;
  std::cout << "[FPGA Handler] Target resource: RIO0" << std::endl;
  
  set_fpga_status(NiFpga_Open(NiFpga_FPGA_RS485_v1_2_Bitfile, NiFpga_FPGA_RS485_v1_2_Signature, "RIO0", 0, &session));
  
  if (NiFpga_IsError(status_)) {
    error_message("[FPGA Handler] Failed to open FPGA session. Error code: " + std::to_string(status_));
    if (status_ == -61200) {
      error_message("[FPGA Handler] Error -61200: FPGA resource not found or bitfile cannot be loaded.");
      error_message("[FPGA Handler] Possible causes:");
      error_message("[FPGA Handler]   1. sbRIO is not powered on or not connected");
      error_message("[FPGA Handler]   2. Bitfile path is incorrect: " + std::string(NiFpga_FPGA_RS485_v1_2_Bitfile));
      error_message("[FPGA Handler]   3. NI-RIO drivers are not properly installed");
      error_message("[FPGA Handler]   4. FPGA resource name 'RIO0' is incorrect for your device");
      error_message("[FPGA Handler]   5. Another process is using the FPGA");
    }
    return;
  }
  important_message("[FPGA Handler] Session opened successfully");

  pwrb_io = PwrbIO(status_, session);
  important_message("[FPGA Handler] PowerBoard I/O Initialized");

  set_fpga_status(NiFpga_ReserveIrqContext(session, &irqContext));
  if (NiFpga_IsError(status_)) {
    error_message("[FPGA Handler] Failed to reserve IRQ context. Error code: " + std::to_string(status_));
    return;
  }
  important_message("[FPGA Handler] IRQ reserved");
}

FpgaHandler::~FpgaHandler() {
  /* unreserve IRQ status to prevent memory leaks */
  set_fpga_status(NiFpga_UnreserveIrqContext(session, &irqContext));

  /* Close the session */
  set_fpga_status(NiFpga_Close(session, 0));
  important_message("[FPGA Handler] Session Closed");

  set_fpga_status(NiFpga_Finalize());
  important_message("[FPGA Handler] Fpga Finalized");
}

void FpgaHandler::set_ni_irq_period(int main_loop_p, int can_loop_p) {
  /* Set up interrupt period (microsecond) */
  /* IRQ 0 */
  set_fpga_status(NiFpga_WriteU32(session, NiFpga_FPGA_RS485_v1_2_ControlU32_IRQ0_period_us, main_loop_p));

  /* IRQ 1 */
  set_fpga_status(NiFpga_WriteU32(session, NiFpga_FPGA_RS485_v1_2_ControlU32_IRQ1_period_us, can_loop_p));
}
