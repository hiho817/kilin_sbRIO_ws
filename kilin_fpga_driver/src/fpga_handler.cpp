#include <fpga_handler.hpp>

ModuleIO::ModuleIO(NiFpga_Status _status, NiFpga_Session _fpga_session, std::string CAN_port_, std::vector<Motor> *motors_list){

    status_ = _status;
    fpga_session_ = _fpga_session;
    motors_list_ = motors_list;

    CAN_timeout_us_ = 1000;
    if (CAN_port_ == "MOD1CAN0")
    {
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
    }
    else if (CAN_port_ == "MOD1CAN1")
    {
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
    }
    else
    {
        std::cout << "[ERROR] CAN_PORT CONFIG ERROR !" << std::endl;
    }
}


// Write FPGA status
void ModuleIO::write_CAN_id_(uint32_t id1, uint32_t id2)
{
    NiFpga_MergeStatus(&status_, NiFpga_WriteU32(fpga_session_, r_CAN_id1_, id1));
    NiFpga_MergeStatus(&status_, NiFpga_WriteU32(fpga_session_, r_CAN_id2_, id2));
}

void ModuleIO::write_CAN_id_fc_(uint32_t id1_fc, uint32_t id2_fc)
{
    NiFpga_MergeStatus(&status_, NiFpga_WriteU32(fpga_session_, r_CAN_id1_FC_, id1_fc));
    NiFpga_MergeStatus(&status_, NiFpga_WriteU32(fpga_session_, r_CAN_id2_FC_, id2_fc));
}

void ModuleIO::read_CAN_id_fc_(uint32_t *fc1, uint32_t *fc2)
{
    NiFpga_MergeStatus(&status_, NiFpga_ReadU32(fpga_session_, r_CAN_id1_FC_, fc1));
    NiFpga_MergeStatus(&status_, NiFpga_ReadU32(fpga_session_, r_CAN_id2_FC_, fc2));
}

void ModuleIO::write_port_select_(const NiFpga_Bool *array)
{
    NiFpga_MergeStatus(&status_, NiFpga_WriteArrayBool(fpga_session_, r_port_select_, array, r_port_select_size_));
}
void ModuleIO::write_tx_data_(const uint8_t *tx_arr1, const uint8_t *tx_arr2)
{
    NiFpga_MergeStatus(&status_, NiFpga_WriteArrayU8(fpga_session_, r_tx_buf_id1_, tx_arr1, r_tx_buf_size_));
    NiFpga_MergeStatus(&status_, NiFpga_WriteArrayU8(fpga_session_, r_tx_buf_id2_, tx_arr2, r_tx_buf_size_));
}
void ModuleIO::write_CAN_transmit_(NiFpga_Bool value)
{
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(fpga_session_, r_CAN_transmit_, value));
}
void ModuleIO::write_timeout_us_(uint32_t value)
{
    NiFpga_MergeStatus(&status_, NiFpga_WriteU32(fpga_session_, r_timeout_us_, value));
}

// Read FPGA status

void ModuleIO::read_rx_data_(uint8_t *rx_arr1, uint8_t *rx_arr2)
{
    NiFpga_MergeStatus(&status_, NiFpga_ReadArrayU8(fpga_session_, r_rx_buf_id1_, rx_arr1, r_rx_buf_size_));
    NiFpga_MergeStatus(&status_, NiFpga_ReadArrayU8(fpga_session_, r_rx_buf_id2_, rx_arr2, r_rx_buf_size_));
}

NiFpga_Bool ModuleIO::read_CAN_complete_()
{
    NiFpga_Bool complete = 0;
    NiFpga_MergeStatus(&status_, NiFpga_ReadBool(fpga_session_, r_CAN_complete_, &complete));
    return complete;
}

NiFpga_Bool ModuleIO::read_CAN_success_()
{
    NiFpga_Bool success = 0;
    NiFpga_MergeStatus(&status_, NiFpga_ReadBool(fpga_session_, r_CAN_success_, &success));
    return success;
}

int16_t ModuleIO::read_CAN_complete_counter_()
{
    int16_t count = 0;
    NiFpga_MergeStatus(&status_, NiFpga_ReadI16(fpga_session_, r_CAN_complete_counter_, &count));
    return count;
}

NiFpga_Bool ModuleIO::read_tx_timeout_()
{
    NiFpga_Bool id1_timeout = 0;
    NiFpga_MergeStatus(&status_, NiFpga_ReadBool(fpga_session_, r_tx_timeout_, &id1_timeout));
    return id1_timeout;
}


NiFpga_Bool ModuleIO::read_rx_timeout_()
{
    NiFpga_Bool id1_timeout = 0;
    NiFpga_MergeStatus(&status_, NiFpga_ReadBool(fpga_session_, r_rx_timeout_, &id1_timeout));
    return id1_timeout;
}

void ModuleIO::CAN_setup(int timeout_us)
{
    write_CAN_id_(motors_list_->at(0).CAN_ID_, motors_list_->at(1).CAN_ID_);
    
    /* select two port to transceive */
    NiFpga_Bool _bool_arr[2] = {1, 1};
    write_port_select_(_bool_arr);

    write_timeout_us_(timeout_us);
}

void ModuleIO::CAN_set_mode(Mode mode)
{
    write_CAN_id_fc_((int)mode, (int)mode);
}

void ModuleIO::CAN_send_command(CAN_txdata txdata_id1, CAN_txdata txdata_id2)
{
    uint8_t txmsg_id1[8];
    uint8_t txmsg_id2[8];
    CAN_txdata txdata1_biased;
    CAN_txdata txdata2_biased;

    txdata1_biased.position_ = txdata_id1.position_ + motor_F_bias;
    txdata1_biased.torque_ = txdata_id1.torque_;
    txdata1_biased.KP_ = txdata_id1.KP_;
    txdata1_biased.KI_ = txdata_id1.KI_;
    txdata1_biased.KD_ = txdata_id1.KD_;

    txdata2_biased.position_ = txdata_id2.position_ + motor_H_bias;
    txdata2_biased.torque_ = txdata_id2.torque_;
    txdata2_biased.KP_ = txdata_id2.KP_;
    txdata2_biased.KI_ = txdata_id2.KI_;
    txdata2_biased.KD_ = txdata_id2.KD_;

    CAN_encode(txmsg_id1, txdata1_biased);
    CAN_encode(txmsg_id2, txdata2_biased);

    uint32_t fc1, fc2;
    read_CAN_id_fc_(&fc1, &fc2);
    
    if (fc1 == 1) txmsg_id1[0] = 255;
    if (fc2 == 1) txmsg_id2[0] = 255;

    write_tx_data_(txmsg_id1, txmsg_id2);
    usleep(100);
    write_CAN_transmit_(1);
}

void ModuleIO::CAN_recieve_feedback(CAN_rxdata *rxdata_id1, CAN_rxdata *rxdata_id2)
{
    uint8_t rxmsg_id1[8];
    uint8_t rxmsg_id2[8];
    read_rx_data_(rxmsg_id1, rxmsg_id2);
    CAN_decode(rxmsg_id1, rxdata_id1);
    CAN_decode(rxmsg_id2, rxdata_id2);

    rxdata_id1->position_ -= motor_F_bias;
    rxdata_id2->position_ -= motor_H_bias;
}

// pack CAN data
void ModuleIO::CAN_encode(uint8_t (&txmsg)[8], CAN_txdata txdata)
{
    int pos_int, torque_int, KP_int, KI_int, KD_int;
    pos_int = float_to_uint(-txdata.position_, P_CMD_MIN, P_CMD_MAX, 16);
    KP_int = float_to_uint(txdata.KP_, KP_MIN, KP_MAX, 12);
    KI_int = float_to_uint(txdata.KI_, KI_MIN, KI_MAX, 12);
    KD_int = float_to_uint(txdata.KD_, KD_MIN, KD_MAX, 12);
    torque_int = float_to_uint(txdata.torque_, T_MIN, T_MAX, 12);

    txmsg[0] = pos_int >> 8;
    txmsg[1] = pos_int & 0xFF;
    txmsg[2] = KP_int >> 4;
    txmsg[3] = ((KP_int & 0x0F) << 4) | (KI_int >> 8);
    txmsg[4] = KI_int & 0xFF;
    txmsg[5] = KD_int >> 4;
    txmsg[6] = ((KD_int & 0x0F) << 4) | (torque_int >> 8);
    txmsg[7] = torque_int & 0xFF;
}

void ModuleIO::CAN_decode(uint8_t (&rxmsg)[8], CAN_rxdata *rxdata)
{
    int pos_raw, vel_raw, torque_raw, cal_raw, ver_raw, mode_raw;

    pos_raw = ((int)(rxmsg[0]) << 8) | rxmsg[1];
    vel_raw = ((int)(rxmsg[2]) << 8) | rxmsg[3];
    torque_raw = ((int)(rxmsg[4]) << 8) | rxmsg[5];
    cal_raw = ((int)(rxmsg[6] & 0x0F));
    ver_raw = ((int)(rxmsg[7] >> 4));
    mode_raw = ((int)(rxmsg[7]& 0x0F));

    rxdata->position_ = -uint_to_float(pos_raw, P_FB_MIN, P_FB_MAX, 16);
    rxdata->velocity_ = uint_to_float(vel_raw, V_MIN, V_MAX, 16);
    rxdata->torque_ = uint_to_float(torque_raw, T_MIN, T_MAX, 16);
    rxdata->version_ = ver_raw;
    rxdata->calibrate_finish_ = cal_raw;
    rxdata->mode_state_ = mode_raw;

    if (mode_raw == _SET_ZERO)rxdata->mode_ = Mode::SET_ZERO;
    else if (mode_raw == _MOTOR_MODE)rxdata->mode_ = Mode::MOTOR;
    else if (mode_raw == _HALL_CALIBRATE)rxdata->mode_ = Mode::HALL_CALIBRATE;
    else if (mode_raw == _REST_MODE)rxdata->mode_ = Mode::REST;
}

int ModuleIO::float_to_uint(float x, float x_min, float x_max, int bits)
{
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float ModuleIO::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

FpgaHandler::FpgaHandler()
{
    status_ = NiFpga_Initialize();
    important_message("[FPGA Handler] Fpga Initialized");

    NiFpga_MergeStatus(&status_, NiFpga_Open(NiFpga_FPGA_RS485_v1_2_Bitfile,
                                             NiFpga_FPGA_RS485_v1_2_Signature, "RIO0", 0, &session_));
    important_message("[FPGA Handler] Session opened");

    NiFpga_MergeStatus(&status_, NiFpga_ReserveIrqContext(session_, &irqContext_));
    important_message("[FPGA Handler] IRQ reserved");
    
    w_pb_digital_ = NiFpga_FPGA_RS485_v1_2_ControlBool_Digital;
    w_pb_signal_ = NiFpga_FPGA_RS485_v1_2_ControlBool_Signal;
    w_pb_power_ = NiFpga_FPGA_RS485_v1_2_ControlBool_Power;

    r_powerboard_data_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU16_Data;
    size_powerboard_data_ = NiFpga_FPGA_RS485_v1_2_IndicatorArrayU16Size_Data;

    // w_vicon_trigger = NiFpga_FPGA_RS485_v1_2_ControlBool_Conn9_2w;

    for (int i = 0; i < 12; i++)
    {
        powerboard_V_list_[i] = 0;
        powerboard_I_list_[i] = 0;
    }
}

FpgaHandler::~FpgaHandler()
{
    /* unreserve IRQ status to prevent memory leaks */
    NiFpga_MergeStatus(&status_, NiFpga_UnreserveIrqContext(session_, &irqContext_));

    /* Close the session */
    NiFpga_MergeStatus(&status_, NiFpga_Close(session_, 0));
    important_message("[FPGA Handler] Session Closed");

    NiFpga_MergeStatus(&status_, NiFpga_Finalize());
    important_message("[FPGA Handler] Fpga Finalized");
}

void FpgaHandler::setIrqPeriod(int main_loop_p, int can_loop_p)
{
    /* Set up interrupt period (microsecond) */
    /* IRQ 0 */
    NiFpga_MergeStatus(
        &status_, NiFpga_WriteU32(session_, NiFpga_FPGA_RS485_v1_2_ControlU32_IRQ0_period_us, main_loop_p));

    /* IRQ 1 */
    NiFpga_MergeStatus(
        &status_, NiFpga_WriteU32(session_, NiFpga_FPGA_RS485_v1_2_ControlU32_IRQ1_period_us, can_loop_p));
}

void FpgaHandler::write_powerboard_(std::vector<bool> *powerboard_state_)
{
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_pb_digital_, powerboard_state_->at(0)));
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_pb_signal_, powerboard_state_->at(1)));
    NiFpga_MergeStatus(&status_, NiFpga_WriteBool(session_, w_pb_power_, powerboard_state_->at(2)));
}

void FpgaHandler::read_powerboard_data_()
{
    uint16_t rx_arr[24];
    // uint16_t *rx_arr = new uint16_t[24];
    NiFpga_MergeStatus(&status_, NiFpga_ReadArrayU16(session_, NiFpga_FPGA_RS485_v1_2_IndicatorArrayU16_Data, rx_arr, NiFpga_FPGA_RS485_v1_2_IndicatorArrayU16Size_Data));

    for (int i = 0; i < 24; i++)
    {
        if (i % 2 == 0)powerboard_I_list_[i / 2] = rx_arr[i] * powerboard_Ifactor[i / 2] + powerboard_Ioffset[i / 2];
        if (i % 2 == 1)powerboard_V_list_[(i - 1) / 2] = rx_arr[i] * powerboard_Vfactor[(i - 1) / 2] + powerboard_Voffset[(i - 1) / 2];
    }
}