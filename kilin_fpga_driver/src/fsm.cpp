#include <fsm.hpp>

/* ModeFsm::ModeFsm(std::vector<HipModule>* _modules,std::vector<LimbModule>* _limb_list, std::vector<bool>* _pb_state, double* pb_v) */
ModeFsm::ModeFsm(std::vector<HipModule>* _modules, std::vector<bool>* _pb_state, double* pb_v)
{
    workingMode_ = Mode::REST;
    prev_workingMode_ = Mode::REST;

    hip_can_list_ = _modules;
		/* limb_list_ = _limb_list; */
    pb_state_ = _pb_state;
    powerboard_voltage = pb_v;

    hall_calibrated = false;
    hall_calibrate_status = 0;
}

void ModeFsm::runFsm(motor_msg::MotorStateStamped& motor_fb_msg, const motor_msg::MotorCmdStamped& motor_cmd_msg)
{
    // position = P_CMD_MAX is to make sure the data received from CONFIG function code is the default one
    switch (workingMode_)
    {
        case Mode::REST: {
            // TODO: pb_state = { digital_switch_, signal_switch_, power_switch_ }
            if (pb_state_->at(2) == true)
            {
                publishMsg(motor_fb_msg);
                for (auto& mod : *hip_can_list_)
                {
                    int index = 0;
                    if (mod.enable_)
                    {
                        mod.txdata_buffer_[0].position_ = 0;
                        mod.txdata_buffer_[0].torque_ = 0;
                        mod.txdata_buffer_[0].KP_ = 0;
                        mod.txdata_buffer_[0].KI_ = 0;
                        mod.txdata_buffer_[0].KD_ = 0;

                        mod.txdata_buffer_[1].position_ = 0;
                        mod.txdata_buffer_[1].torque_ = 0;
                        mod.txdata_buffer_[1].KP_ = 0;
                        mod.txdata_buffer_[1].KI_ = 0;
                        mod.txdata_buffer_[1].KD_ = 0;
                    }
                }
            }
        }
        break;

        case Mode::SET_ZERO: {
            if (pb_state_->at(2) == true)
            {
                for (int i = 0; i < 2; i++)
                {
                    if (hip_can_list_->at(i).enable_)
                    {
                        hip_can_list_->at(i).io_.motor_F_bias = 0;
                        hip_can_list_->at(i).io_.motor_H_bias = 0;
                    }
                }

                publishMsg(motor_fb_msg);
                for (auto& mod : *hip_can_list_)
                {
                    if (mod.enable_)
                    {
                        mod.txdata_buffer_[0].position_ = P_CMD_MAX;
                        mod.txdata_buffer_[0].torque_ = 0;
                        mod.txdata_buffer_[0].KP_ = 0;
                        mod.txdata_buffer_[0].KI_ = 0;
                        mod.txdata_buffer_[0].KD_ = 0;
                        
                        mod.txdata_buffer_[1].position_ = P_CMD_MAX;
                        mod.txdata_buffer_[1].torque_ = 0;
                        mod.txdata_buffer_[1].KP_ = 0;
                        mod.txdata_buffer_[1].KI_ = 0;
                        mod.txdata_buffer_[1].KD_ = 0;
                    }
                }
            }
        }
        break;
        
        case Mode::HALL_CALIBRATE: {
            int module_enabled = 0;

            for (int i = 0; i < 2; i++)
            {
                if (hip_can_list_->at(i).enable_)
                {
                    hip_can_list_->at(i).txdata_buffer_[0].position_ = 0;
                    hip_can_list_->at(i).txdata_buffer_[0].torque_ = 0;
                    hip_can_list_->at(i).txdata_buffer_[0].KP_ = 0;
                    hip_can_list_->at(i).txdata_buffer_[0].KI_ = 0;
                    hip_can_list_->at(i).txdata_buffer_[0].KD_ = 0;

                    hip_can_list_->at(i).txdata_buffer_[1].position_ = 0;
                    hip_can_list_->at(i).txdata_buffer_[1].torque_ = 0;
                    hip_can_list_->at(i).txdata_buffer_[1].KP_ = 0;
                    hip_can_list_->at(i).txdata_buffer_[1].KI_ = 0;
                    hip_can_list_->at(i).txdata_buffer_[1].KD_ = 0;
                    module_enabled++;
                }
            }
            switch (hall_calibrate_status)
            {
                case -1:{
                    switchMode(Mode::REST);
                }
                break;
            
                case 0:{
                    int cal_cnt = 0;
                    for (int i = 0; i < 2; i++)
                    {
                        if (hip_can_list_->at(i).enable_ && hip_can_list_->at(i).rxdata_buffer_[0].calibrate_finish_ == 2 && hip_can_list_->at(i).rxdata_buffer_[1].calibrate_finish_ == 2) cal_cnt++;
                    }
                    if (cal_cnt == module_enabled && measure_offset == 0) hall_calibrate_status++;
                    else if (cal_cnt == module_enabled && measure_offset == 1) hall_calibrate_status = -1;
                }
                break;

                case 1:{
                    for (int i = 0; i < 2; i++)
                    {
                        if (hip_can_list_->at(i).enable_)
                        {
                            hip_can_list_->at(i).CAN_rx_timedout_[0] = false;
                            hip_can_list_->at(i).CAN_rx_timedout_[1] = false;
                            hip_can_list_->at(i).CAN_tx_timedout_[0] = false;
                            hip_can_list_->at(i).CAN_tx_timedout_[1] = false;

                            hip_can_list_->at(i).io_.motor_F_bias = hip_can_list_->at(i).Motor_F_bias;
                            hip_can_list_->at(i).io_.motor_H_bias = hip_can_list_->at(i).Motor_H_bias;

                            cal_command[i][0] = - hip_can_list_->at(i).Motor_F_bias;
                            hip_can_list_->at(i).txdata_buffer_[0].position_ = - hip_can_list_->at(i).Motor_F_bias;
                            cal_dir_[i][0] = 1;

                            cal_command[i][1] = - hip_can_list_->at(i).Motor_H_bias;
                            hip_can_list_->at(i).txdata_buffer_[1].position_ = - hip_can_list_->at(i).Motor_H_bias;
                            cal_dir_[i][1] = -1;
                        }
                    }
                    hall_calibrate_status++;
                }
                break;

                case 2:{
                    int finish_cnt = 0;
                    for (int i = 0; i < 2; i++)
                    {
                        if (hip_can_list_->at(i).enable_){
                            for (int j = 0; j < 2; j++)
                            {
                                double errj = 0;
                                errj = cal_command[i][j];

                                if (fabs(errj) < cal_tol_)
                                {
                                    hip_can_list_->at(i).txdata_buffer_[j].position_ = 0;
                                    hip_can_list_->at(i).txdata_buffer_[j].torque_ = 0;
                                    hip_can_list_->at(i).txdata_buffer_[j].KP_ = 0;
                                    hip_can_list_->at(i).txdata_buffer_[j].KI_ = 0;
                                    hip_can_list_->at(i).txdata_buffer_[j].KD_ = 0;
                                    finish_cnt++;
                                }
                                else
                                {
                                    hip_can_list_->at(i).io_.write_CAN_id_fc_((int)Mode::CONTROL, (int)Mode::CONTROL);
                                    cal_command[i][j] += cal_dir_[i][j] * cal_vel_ * dt_;
                                    hip_can_list_->at(i).txdata_buffer_[j].position_ = cal_command[i][j];
                                    hip_can_list_->at(i).txdata_buffer_[j].torque_ = 0;
                                    hip_can_list_->at(i).txdata_buffer_[j].KP_ = 50;
                                    hip_can_list_->at(i).txdata_buffer_[j].KI_ = 0;
                                    hip_can_list_->at(i).txdata_buffer_[j].KD_ = 1.5;
                                }
                            }
                        }
                    }
                    if (finish_cnt == 2 * module_enabled) hall_calibrate_status++;
                }
                break;

                case 3:{
                    hall_calibrated = true;
                    hall_calibrate_status = 0;
                    switchMode(Mode::MOTOR);
                }
                break;
            }
        }
        break;

        case Mode::MOTOR: {
            /* Pubish feedback data from Motors */
            publishMsg(motor_fb_msg);
            int index = 0;
            for (auto& mod : *hip_can_list_)
            {
                if (mod.enable_)
                {
                    /* Subscribe command from other nodes */
                    // initialize message
                    // update
                    if (*NO_CAN_TIMEDOUT_ERROR_ && *NO_SWITCH_TIMEDOUT_ERROR_ )
                    {
                        switch (index)
                        {
                            /*********************************************/
                            // case 0:                  case 1:
                            // MOD1CAN0 (module_L)      MOD2CAN1 (module_R)
                            //  CAN1 LF (module_a)       CAN1 RF (module_b)
                            //  CAN2 LH (module_d)       CAN2 RH (module_c)
                            /*********************************************/
                            //  +-------+--------+       +--------+-------+
                            //  | CAN0-port0     |       | CAN1-port0     |
                            //  | module_a (LF)  |       | module_b (RF)  |
                            //  | Left Front Hip |       | Right Front Hip|
                            //  +----------------+       +----------------+
                            //  
                            //  +----------------+       +----------------+
                            //  | CAN0-port1     |       | CAN1-port1     |
                            //  | module_d (LH)  |       | module_c (RH)  |
                            //  | Left Hind Hip  |       | Right Hind Hip |
                            //  +----------------+       +----------------+
                            /*********************************************/
                            case 0:
                            {   
				// std::cout<< "motor_a_position_msg: " << motor_cmd_msg.module_a().hip().position() << std::endl;
				mod.txdata_buffer_[0].position_ = motor_cmd_msg.module_a().hip().position();
                                mod.txdata_buffer_[0].torque_ = motor_cmd_msg.module_a().hip().torque();
                                mod.txdata_buffer_[0].KP_ = motor_cmd_msg.module_a().hip().kp();
                                mod.txdata_buffer_[0].KI_ = motor_cmd_msg.module_a().hip().ki();
                                mod.txdata_buffer_[0].KD_ = motor_cmd_msg.module_a().hip().kd();

                                mod.txdata_buffer_[1].position_ = motor_cmd_msg.module_d().hip().position();
                                mod.txdata_buffer_[1].torque_ = motor_cmd_msg.module_d().hip().torque();
                                mod.txdata_buffer_[1].KP_ = motor_cmd_msg.module_d().hip().kp();
                                mod.txdata_buffer_[1].KI_ = motor_cmd_msg.module_d().hip().ki();
                                mod.txdata_buffer_[1].KD_ = motor_cmd_msg.module_d().hip().kd();
                            }
                            break;

                            case 1:
                            {
                                mod.txdata_buffer_[0].position_ = motor_cmd_msg.module_b().hip().position();
                                mod.txdata_buffer_[0].torque_ = motor_cmd_msg.module_b().hip().torque();
                                mod.txdata_buffer_[0].KP_ = motor_cmd_msg.module_b().hip().kp();
                                mod.txdata_buffer_[0].KI_ = motor_cmd_msg.module_b().hip().ki();
                                mod.txdata_buffer_[0].KD_ = motor_cmd_msg.module_b().hip().kd();

                                mod.txdata_buffer_[1].position_ = motor_cmd_msg.module_c().hip().position();
                                mod.txdata_buffer_[1].torque_ = motor_cmd_msg.module_c().hip().torque();
                                mod.txdata_buffer_[1].KP_ = motor_cmd_msg.module_c().hip().kp();
                                mod.txdata_buffer_[1].KI_ = motor_cmd_msg.module_c().hip().ki();
                                mod.txdata_buffer_[1].KD_ = motor_cmd_msg.module_c().hip().kd();
                            }
                            break;
                        }
                    }
                }
                index++;
            }

        }
        break;

        case Mode::CONFIG: {
            // for debug
        }
        break;
    }       
}

bool ModeFsm::switchMode(Mode next_mode)
{
    int mode_switched_cnt = 0;
    int module_enabled = 0;
    bool success = false;
    Mode next_mode_switch = next_mode;

    for (int i = 0; i < 2; i++)
    {
        if (hip_can_list_->at(i).enable_) module_enabled++;
    }
    

    double time_elapsed = 0;
    while (1)
    {
        if (mode_switched_cnt == module_enabled)
        {
            prev_workingMode_ = workingMode_;
            workingMode_ = next_mode_switch;
            success = true;
            break;
        }
        else if (time_elapsed > 3)
        {
            /* Timeout */
            success = false;
            break;
        }
        else  mode_switched_cnt = 0;

        for (int i = 0; i < 2; i++)
        {
            if (hip_can_list_->at(i).enable_)
            {
                hip_can_list_->at(i).io_.write_CAN_id_fc_((int)next_mode_switch, (int)next_mode_switch);
                hip_can_list_->at(i).io_.write_CAN_transmit_(1);
                hip_can_list_->at(i).io_.CAN_recieve_feedback(&hip_can_list_->at(i).rxdata_buffer_[0],
                                                              &hip_can_list_->at(i).rxdata_buffer_[1]);
                if ((next_mode_switch == Mode::SET_ZERO
                    && (int)hip_can_list_->at(i).rxdata_buffer_[0].position_ <= 0.01
                    && (int)hip_can_list_->at(i).rxdata_buffer_[0].position_ >= -0.01)
                    || 
                    ((int)hip_can_list_->at(i).rxdata_buffer_[0].mode_ == (int)next_mode_switch
                    && (int)hip_can_list_->at(i).rxdata_buffer_[1].mode_ == (int)next_mode_switch))
                {
                    mode_switched_cnt++;
                }
            }
        }
        
        time_elapsed += 0.01;
        usleep(1e4);
    }

    for (int i = 0; i < 2; i++)
    {
        if (hip_can_list_->at(i).enable_){
            if (workingMode_ == Mode::MOTOR) hip_can_list_->at(i).io_.write_CAN_id_fc_((int)Mode::CONTROL, (int)Mode::CONTROL);
            else hip_can_list_->at(i).io_.write_CAN_id_fc_((int)Mode::CONFIG, (int)Mode::CONFIG);
        }
    }

    return success;
}

void ModeFsm::publishMsg(motor_msg::MotorStateStamped& motor_fb_msg)
{
    int index = 0;
    for (auto& mod : *hip_can_list_)
    {
        if (mod.enable_)
        {
            switch (index)
            {
                case 0: // CAN0 (module_L)
                {
                    /* Publish feedback data from Motors_F */
                    motor_fb_msg.mutable_module_a()->mutable_hip()->set_position(mod.rxdata_buffer_[0].position_);
                    motor_fb_msg.mutable_module_a()->mutable_hip()->set_velocity(mod.rxdata_buffer_[0].velocity_);
                    motor_fb_msg.mutable_module_a()->mutable_hip()->set_torque(mod.rxdata_buffer_[0].torque_*mod.txdata_buffer_[0].KT_); //torque F 
                    /* Publish feedback data from Motors_H */
                    motor_fb_msg.mutable_module_d()->mutable_hip()->set_position(mod.rxdata_buffer_[1].position_);
                    motor_fb_msg.mutable_module_d()->mutable_hip()->set_velocity(mod.rxdata_buffer_[1].velocity_);
                    motor_fb_msg.mutable_module_d()->mutable_hip()->set_torque(mod.rxdata_buffer_[1].torque_*mod.txdata_buffer_[1].KT_); //torque H
                }
                break;

                case 1: // CAN1 (module_R)
                {
                    /* Publish feedback data from Motors_F */
                    motor_fb_msg.mutable_module_b()->mutable_hip()->set_position(mod.rxdata_buffer_[0].position_);
                    motor_fb_msg.mutable_module_b()->mutable_hip()->set_velocity(mod.rxdata_buffer_[0].velocity_);
                    motor_fb_msg.mutable_module_b()->mutable_hip()->set_torque(mod.rxdata_buffer_[0].torque_*mod.txdata_buffer_[0].KT_); //torque F 
                    /* Publish feedback data from Motors_H */
                    motor_fb_msg.mutable_module_c()->mutable_hip()->set_position(mod.rxdata_buffer_[1].position_);
                    motor_fb_msg.mutable_module_c()->mutable_hip()->set_velocity(mod.rxdata_buffer_[1].velocity_);
                    motor_fb_msg.mutable_module_c()->mutable_hip()->set_torque(mod.rxdata_buffer_[1].torque_*mod.txdata_buffer_[1].KT_); //torque H
                }
                break;
            }
        }
        else
        {
            switch (index)
            {
                case 0: // CAN0 (module_L)
                {
                    /* Publish feedback data from Motors_F */
                    motor_fb_msg.mutable_module_a()->mutable_hip()->set_position(0);
                    motor_fb_msg.mutable_module_a()->mutable_hip()->set_velocity(0);
                    motor_fb_msg.mutable_module_a()->mutable_hip()->set_torque(0); //torque F 
                    /* Publish feedback data from Motors_H */
                    motor_fb_msg.mutable_module_d()->mutable_hip()->set_position(0);
                    motor_fb_msg.mutable_module_d()->mutable_hip()->set_velocity(0);
                    motor_fb_msg.mutable_module_d()->mutable_hip()->set_torque(0); //torque H
                }
                break;

                case 1: // CAN0 (module_R)
                {
                    /* Publish feedback data from Motors_F */
                    motor_fb_msg.mutable_module_b()->mutable_hip()->set_velocity(0);
                    motor_fb_msg.mutable_module_b()->mutable_hip()->set_position(0);
                    motor_fb_msg.mutable_module_b()->mutable_hip()->set_torque(0); //torque F 
                    /* Publish feedback data from Motors_H */
                    motor_fb_msg.mutable_module_c()->mutable_hip()->set_position(0);
                    motor_fb_msg.mutable_module_c()->mutable_hip()->set_velocity(0);
                    motor_fb_msg.mutable_module_c()->mutable_hip()->set_torque(0); //torque H
                }
                break;
            }
        }
        index++;
    }

		/* index = 0 ; */
		/* for (auto& limb : *limb_list_) */
		/* { */
		/* 	if(limb.enable_) */
		/* 	{ */
		/* 		switch(index) */
		/* 		{ */
		/* 		} */
		/* 	} */
		/* 	else */
		/* 	{ */
		/* 		switch(index) */
		/* 		{ */
		/* 				case 0: // RS458 0 */
		/* 				{ */
		/* 						/1* Publish feedback data from Motors_steering *1/ */
		/* 						motor_fb_msg.mutable_module_a()->mutable_steering()->set_position(0); */
		/* 						motor_fb_msg.mutable_module_a()->mutable_steering()->set_velocity(0); */
		/* 						motor_fb_msg.mutable_module_a()->mutable_steering()->set_torque(0); */
		/* 						/1* Publish feedback data from Motors_hub *1/ */
		/* 						motor_fb_msg.mutable_module_a()->mutable_hub()->set_position(0); */
		/* 						motor_fb_msg.mutable_module_a()->mutable_hub()->set_velocity(0); */
		/* 						motor_fb_msg.mutable_module_a()->mutable_hub()->set_torque(0); */
		/* 				} */
		/* 				break; */
		/* 		} */
		/* 	} */
		/* } */


}
