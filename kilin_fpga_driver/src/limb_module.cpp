#include <limb_module.hpp>

LimbModule::LimbModule(std::string _label, YAML::Node _config, NiFpga_Status _status, NiFpga_Session _fpga_session)
{
	    label_ = _label;
			/* config_ = _config; */
			enable_ = false;

			/* load_config(); */

			RS485_tx_timedout_[0] = false;
			RS485_tx_timedout_[1] = false;
			RS485_rx_timedout_[0] = false;
			RS485_rx_timedout_[1] = false;

			RS485_mtr_timedout[0] = false;
			RS485_mtr_timedout[1] = false;

			RS485_module_timedout = false;

}

void LimbModule::load_config()
{
	Motor motor_steer;
	Motor motor_wheel;
	/* RS485_timeout_us = config["RS485_Timeout_us"].as<int>(); */
}

void LimbModule::RS485_timeoutCheck()
{}

double deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}
