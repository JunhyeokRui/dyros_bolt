#include "dyros_bolt_real_robot/odrive_socketcan_master.h"

int shm_id_;
SHMmsgs *shm_msgs_;
int Q_START = -1;
int PART_CAN_DOF = -1;
float k_tau[MODEL_DOF] = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
float torque_desired_[MODEL_DOF];
float extencoder_offset_[MODEL_DOF];
float imu_data_quat[4]; 
float imu_accelometer_[3]; // current accelometer values
float imu_angular_velocity[3];
float q_[MODEL_DOF];      // sendstate
float q_dot_[MODEL_DOF];  // sendstate
float torque_[MODEL_DOF]; // sendstate
float q_ext_[MODEL_DOF];
float q_ext_offset_[MODEL_DOF] ;

bool extencoder_init_flag_;
unsigned int joint_id_[MODEL_DOF];
unsigned int joint_id_inversed_[MODEL_DOF];
unsigned int control_mask_[MODEL_DOF];

volatile bool de_zp_switch;
volatile bool de_zp;
volatile bool de_zp_sequence;
bool fz_check = false;
bool check_commutation = true;
bool check_commutation_first = true;
bool motorCalibDonePrint = false;
bool encoderCalibDonePrint = false;
bool encoderResetDonePrint = false;

odrive::ODriveSocketCan odrv;    

DyrosBoltInitArgs g_init_args;


bool initDyrosBoltArgs(const DyrosBoltInitArgs &args)
{
    
    
    Q_START = args.q_start_;
    PART_CAN_DOF = args.can_slave_num;

    extencoder_init_flag_ = false;

    init_shm(shm_msg_key, shm_id_, &shm_msgs_);

    shm_msgs_->motorCalibSwitch = false;
    shm_msgs_->encoderCalibSwitch = false;
    shm_msgs_->encoderResetSwitch = false;


    if (shm_msgs_->shutdown == true)
    {
        std::cout << "SHUTDOWN -> TRUE" << std::endl;
        std::cout << "CAN : shm_reset" << std::endl;
    }

    return true;
}

bool initDyrosBoltSystem(const DyrosBoltInitArgs &args)
{
    



    return true;
}

void shutdownSystem()
{
    shm_msgs_->shutdown = true;
}

void cleanupDyrosBoltSystem()
{
    deleteSharedMemory(shm_id_, shm_msgs_);
}

void *ethercatThread1(void *data)
{
    DyrosBoltInitArgs *init_args = (DyrosBoltInitArgs *)data;

    printf("ODRV : START Initialization Mode\n");
    // while(shm_msgs_->initializeDyrosBolt == false)
    // {
        
    // }

    // switch (requestState) {
    //     case 1:
    //         odrv.disengage();
    //         break;
    //     case 2:
    //         for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
    //             odrv.requestODriveCmd(i, odrive::ODriveCommandId::ESTOP_MESSAGE);
    //         }
    //         break;    
    //     case 4:
    //         for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
    //             odrv.setAxisRequestedState(odrv.axis_can_ids_list[i], odrive::ODriveAxisState::MOTOR_CALIBRATION);
    //         }
    //         break;
    //     case 7:
    //         for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
    //             odrv.setAxisRequestedState(odrv.axis_can_ids_list[i], odrive::ODriveAxisState::ENCODER_OFFSET_CALIBRATION);
    //         }
    //         break;
    //     case 8:
    //         odrv.engage();
    //         // for(int i=0; i< DyrosBoltModel::HW_TOTAL_DOF / 2 - 1; i++)
    //         // {
    //         //     // odrv.setInputTorque(i, 0);
    //         //     // odrv.setInputTorque(i+3,0);
    //         // }
    //         break;
    //     case 16:
    //         for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
    //             odrv.requestODriveCmd(i, odrive::ODriveCommandId::REBOOT_ODRIVE);
    //         }
    //         break;
    //     case 19:
    //         for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
    //             odrv.resetEncoder(i, odrive::ODriveCommandId::SET_ABSOLUTE_POSITION);
    //         }
    //         break;    
    // }

    if (shm_msgs_->shutdown)
        printf("Shutdown Command Before Start\n");

    while (!shm_msgs_->shutdown)
    {
        
        if (!shm_msgs_->shutdown)
            printf(" REALROBOT : Control Mode Start ... \n");
        
        readDevice();
        if(shm_msgs_->odrvTorqueOn){
            odrv.engage();
            printf("ODRV : TORQUE ON ..\n");
            shm_msgs_->odrvTorqueOn = false;
        }
        if(shm_msgs_->odrvTorqueOff){
            odrv.disengage();
            printf("ODRV : TORQUE OFF ..\n");
            shm_msgs_->odrvTorqueOff = false;
        }

//rui - /*get motordrive state*/
        //ANCHOR - motor calibration
        if(shm_msgs_->motorCalibSwitch){
            printf("Motor Calib Pressed\n");
            std::cout << "odrv.axis_can_ids_list.size() : " << odrv.axis_can_ids_list.size() << std::endl;

            for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
                    odrv.setAxisRequestedState(odrv.axis_can_ids_list[i], odrive::ODriveAxisState::MOTOR_CALIBRATION);
            }

            for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
                if(odrv.axis_current_state[i] != 0 || odrv.axis_error[i] == 0){
                    printf("Motor Calibration in Progress ...\n");
                }
            }
            for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
                if(odrv.axis_current_state[i] == 0 && odrv.axis_error[i] == 0){
                    printf("Motor Calibration Done\n");
                    shm_msgs_->motorCalibSwitch = false;
                    motorCalibDonePrint = true;
                }
            }
            
        }
        if(motorCalibDonePrint && !encoderCalibDonePrint){
            printf("Motor Calib Done\n");
        }

        //ANCHOR - encoder calibration
        if (shm_msgs_->encoderCalibSwitch)
        {
            printf("Encoder Calib Pressed\n");
            for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
                odrv.setAxisRequestedState(odrv.axis_can_ids_list[i], odrive::ODriveAxisState::ENCODER_OFFSET_CALIBRATION);
            }
            
            for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
                if(odrv.axis_current_state[i] != 0 || odrv.axis_error[i] == 0){
                    printf("Encoder Calibration in Progress ...\n");    
                }
            }
            for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
                if(odrv.axis_current_state[i] == 0 && odrv.axis_error[i] == 0){
                    printf("Encoder Calibration Done \n");    
                    shm_msgs_->encoderCalibSwitch = false;
                    encoderCalibDonePrint = true;
                }
            }
        }
        if(encoderCalibDonePrint && !encoderResetDonePrint){
            printf("encoder Calib Done\n");
        }

        // for(int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
        //     if(odrv.axis_current_state[i] != 8) {
        //         return false;
        //     }
        // }

        if (shm_msgs_->encoderResetSwitch)
        {
            printf("Encoder Reset Pressed\n");
            for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
                odrv.resetEncoder(i, odrive::ODriveCommandId::SET_ABSOLUTE_POSITION);
            }
            shm_msgs_->encoderResetSwitch = false;
            encoderResetDonePrint = true;
        }
        if(encoderResetDonePrint){
            printf("encoder Reset Done\n");
        }

        // ts.tv_nsec += init_args->period_ns + toff;
        // if (ts.tv_nsec >= SEC_IN_NSEC)
        // {
        //     ts.tv_sec++;
        //     ts.tv_nsec -= SEC_IN_NSEC;
        // }
        // ec_send_processdata();
        // clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);

        // cycle_count++;

        // wkc = ec_receive_processdata(EC_PACKET_TIMEOUT);

        // control_time_real_ = cycle_count * init_args->period_ns / 1000000000.0;

        // for (int i = 0; i < ec_slavecount; i++)
        // {
        //     // printf("%d\t",rxPDO[i]->statusWord);

        //     elmost[i].state = getElmoState(rxPDO[i]->statusWord);

        //     if (elmost[i].state != elmost[i].state_before)
        //     {
        //         state_elmo_[JointMap2[START_N + i]] = elmost[i].state;

        //         if (elmost[i].first_check)
        //         {
        //             if (elmost[i].state == ELMO_NOTFAULT)
        //             {
        //                 elmost[i].commutation_required = true;
        //             }
        //             else if (elmost[i].state == ELMO_FAULT)
        //             {
        //                 // printf("slave : " << i << " commutation check complete at first\n");
        //                 elmost[i].commutation_not_required = true;
        //             }
        //             else if (elmost[i].state == ELMO_OPERATION_ENABLE)
        //             {
        //                 // printf("slave : " << i << " commutation check complete with operation enable\n");
        //                 elmost[i].commutation_not_required = true;
        //                 elmost[i].commutation_ok = true;
        //             }
        //             else
        //             {
        //                 // printf("first missing : slave : " << i << " state : " << elmost[i].state << '\n';
        //             }
        //             elmost[i].first_check = false;
        //         }
        //         else
        //         {
        //             if (elmost[i].state == ELMO_OPERATION_ENABLE)
        //             {
        //                 // printf("slave : %d commutation check complete with operation enable 2\n",i);
        //                 elmost[i].commutation_ok = true;
        //                 elmost[i].commutation_required = false;
        //             }
        //         }
        //         query_check_state = true;
        //     }
        //     elmost[i].state_before = elmost[i].state;
        // }
        
//rui - /*check commutation*/


        // if (check_commutation)
        // {
        //     if (check_commutation_first)
        //     {
        //         if (init_args->ecat_device == 0)
        //         {
        //             printf("Commutation Status : \n");
        //             for (int i = 0; i < ec_slavecount; i++)
        //                 printf("--");
        //             printf("\n");
        //             for (int i = 0; i < ec_slavecount; i++)
        //                 printf("%2d", (i - i % 10) / 10);
        //             printf("\n");
        //             for (int i = 0; i < ec_slavecount; i++)
        //                 printf("%2d", i % 10);
        //             printf("\n");
        //             printf("\n");
        //             printf("\n");
        //         }
        //         check_commutation_first = false;
        //     }
        //     if (query_check_state)
        //     {
        //         if (init_args->ecat_device == 0)
        //         {
        //             printf("\x1b[A\x1b[A\33[2K\r");
        //             for (int i = 0; i < ec_slavecount; i++)
        //             {
        //                 if (elmost[i].state == ELMO_OPERATION_ENABLE)
        //                 {
        //                     printf("%s%2d%s", cgreen, elmost[i].state, creset);
        //                 }
        //                 else
        //                 {
        //                     printf("%2d", elmost[i].state);
        //                 }
        //             }
        //             printf("\n");
        //             for (int i = 0; i < ec_slavecount; i++)
        //                 printf("--");
        //             printf("\n");
        //             fflush(stdout);
        //         }
        //         query_check_state = false;
        //     }
        // }

        // bool waitop = true;
        // for (int i = 0; i < ec_slavecount; i++)
        //     waitop = waitop && elmost[i].commutation_ok;

        // if (waitop)
        // {
        //     static bool pub_once = true;

        //     if (pub_once)
        //     {

        //         long commutation_time = getTimeDiff(ts_start);

        //         int commutation_min_time = 1000;

        //         if (commutation_time / 1e6 < commutation_min_time)
        //         {

        //             de_commutation_done = true;
        //             check_commutation = false;
        //             if (init_args->verbose)
        //                 printf("ELMO %d : Load ZP ... \n", init_args->ecat_device);
        //         }
        //         else
        //         {

        //             printf("ELMO %d : All slaves Operational in %f ms, > %d\n", init_args->ecat_device, commutation_time / 1e6, commutation_min_time);

        //             if (saveCommutationLog())
        //             {
        //                 if (init_args->verbose)
        //                     printf("\nELMO %d : Commutation is done, logging success\n", init_args->ecat_device);
        //             }
        //             else
        //             {
        //                 printf("\nELMO %d : Commutation is done, logging failed\n", init_args->ecat_device);
        //             }
        //             de_commutation_done = true;
        //             check_commutation = false;
        //         }

        //         pub_once = false;
        //     }
        // }

        // if (de_commutation_done)
        // {
        //     static bool pub_once = true;
        //     if (pub_once)
        //     {

        //         if (loadZeroPoint())
        //         {
                    
        //             break;
        //         }
        //         else
        //         {
        //             printf("ELMO %d : ZeroPoint load failed. Ready to Search Zero Point \n", init_args->ecat_device);
        //             de_zp_sequence = true;
        //         }
        //         pub_once = false;
        //     }
        // }

        // if (shm_msgs_->force_load_saved_signal)
        // {
        //     loadZeroPoint(true);
        //     printf("ELMO 1 : force load ZP\n");
        //     break;
        // }

        // bool waitcm = true;
        // for (int i = 0; i < ec_slavecount; i++)
        //     waitcm = waitcm && elmost[i].commutation_not_required;

        // if (waitcm)
        // {
        //     if (wait_kill_switch)
        //     {
        //         if (init_args->verbose)
        //             printf("ELMO %d : Commutation state OK\n", init_args->ecat_device);
        //         // loadCommutationLog();
        //         loadZeroPoint();
        //         wait_kill_switch = false;
        //         check_commutation = false;
        //     }
        //     if (wait_cnt == 200)
        //     {
        //         printf("ELMO %d : slaves status are not OP! maybe kill switch is on?\n", init_args->ecat_device);
        //     }

        //     wait_cnt++;
        // }
        // else
        // {
        //     int total_commutation_cnt = 0;
        //     for (int i = 0; i < ec_slavecount; i++)
        //     {
        //         if (elmost[i].commutation_required)
        //         {
        //             total_commutation_cnt++;
        //             if (total_commutation_cnt < 4)
        //                 controlWordGenerate(rxPDO[i]->statusWord, txPDO[i]->controlWord);
        //             txPDO[i]->maxTorque = (uint16)400; // originaly 1000
        //         }
        //     }
        // }

//rui - /*homming*/
        // if (wkc >= expectedWKC)
        // {
        //     for (int slave = 1; slave <= ec_slavecount; slave++)
        //     {
        //         if (!elmost[slave - 1].commutation_required)
        //         {
        //             if (controlWordGenerate(rxPDO[slave - 1]->statusWord, txPDO[slave - 1]->controlWord))
        //             {
        //                 reachedInitial[slave - 1] = true;
        //             }
        //             if (reachedInitial[slave - 1])
        //             {
        //                 q_elmo_[START_N + slave - 1] = rxPDO[slave - 1]->positionActualValue * CNT2RAD[START_N + slave - 1] * elmo_axis_direction[START_N + slave - 1]; // - q_zero_elmo_[START_N + slave - 1];

        //                 if (START_N + slave - 1 == R_HipYaw_Joint)
        //                 {
        //                     grav_signal = (((uint32_t)ec_slave[slave].inputs[6]) & ((uint32_t)1));
        //                     pos_signal = (((uint32_t)ec_slave[slave].inputs[6]) & ((uint32_t)2));
        //                 }

        //                 hommingElmo[START_N + slave - 1] =
        //                     (((uint32_t)ec_slave[slave].inputs[6]) & ((uint32_t)1));
        //                 q_dot_elmo_[START_N + slave - 1] =
        //                     (((int32_t)ec_slave[slave].inputs[10]) +
        //                      ((int32_t)ec_slave[slave].inputs[11] << 8) +
        //                      ((int32_t)ec_slave[slave].inputs[12] << 16) +
        //                      ((int32_t)ec_slave[slave].inputs[13] << 24)) *
        //                     CNT2RAD[START_N + slave - 1] * elmo_axis_direction[START_N + slave - 1];
        //                 torque_elmo_[START_N + slave - 1] =
        //                     (int16_t)(((int16_t)ec_slave[slave].inputs[14]) +
        //                               ((int16_t)ec_slave[slave].inputs[15] << 8));
        //                 q_ext_elmo_[START_N + slave - 1] =
        //                     (((int32_t)ec_slave[slave].inputs[16]) +
        //                      ((int32_t)ec_slave[slave].inputs[17] << 8) +
        //                      ((int32_t)ec_slave[slave].inputs[18] << 16) +
        //                      ((int32_t)ec_slave[slave].inputs[19] << 24) - q_ext_mod_elmo_[START_N + slave - 1]) *
        //                     EXTCNT2RAD[START_N + slave - 1] * elmo_ext_axis_direction[START_N + slave - 1];

        //                 if (q_ext_elmo_[START_N + slave - 1] > 3.141592)
        //                 {
        //                     q_ext_elmo_[START_N + slave - 1] -= 3.141592 * 2;
        //                 }
        //                 else if (q_ext_elmo_[START_N + slave - 1] < -3.141592)
        //                 {
        //                     q_ext_elmo_[START_N + slave - 1] += 3.141592 * 2;
        //                 }

        //                 if (START_N + slave == 1 || START_N + slave == 2 || START_N + slave == 7 || START_N + slave == 19 || START_N + slave == 20 || START_N + slave == 16)
        //                 {
        //                     hommingElmo[START_N + slave - 1] = !hommingElmo[START_N + slave - 1];
        //                 }
        //                 txPDO[slave - 1]->maxTorque = (uint16)500; // originaly 1000
        //             }
        //         }
        //     }
        // }

        // for (int i = 0; i < ec_slavecount; i++)
        // {
        //     q_[JointMap2[START_N + i]] = q_elmo_[START_N + i];
        //     q_dot_[JointMap2[START_N + i]] = q_dot_elmo_[START_N + i];
        //     torque_[JointMap2[START_N + i]] = torque_elmo_[START_N + i];

        //     if (g_init_args.ecat_device == 1)
        //         q_ext_[JointMap2[START_N + i]] = q_desired_elmo_[START_N + i];
        //     else
        //         q_ext_[JointMap2[START_N + i]] = q_ext_elmo_[START_N + i];

        // }

        if (shm_msgs_->bolt_init_signal)
        {
            de_zp_switch = true;
            shm_msgs_->bolt_init_signal = false;
        }


        sendJointStatus();

        // if (de_zp_sequence)
        // {
        //     static bool zp_upper = false;
        //     static bool zp_lower = false;

        //     if (de_zp_switch)
        //     {
        //         printf("ODRV %d : Starting zp\n", init_args->ecat_device);
                
        //         elmofz[R_Shoulder3_Joint].findZeroSequence = 7;
        //         elmofz[R_Shoulder3_Joint].initTime = control_time_real_;
        //         elmofz[L_Shoulder3_Joint].findZeroSequence = 7;
        //         elmofz[L_Shoulder3_Joint].initTime = control_time_real_;

        //         for (int i = 0; i < ec_slavecount; i++)
        //             hommingElmo_before[START_N + i] = hommingElmo[START_N + i];

        //         de_zp = true;
        //         de_zp_switch = false;
        //     }

        //     if (de_zp)
        //     {
                
        //         for (int i = 0; i < 18; i++)
        //         {
        //             findZeroPoint(fz_group1[i], control_time_real_);
        //         }
                
        //         for (int i = 0; i < ec_slavecount; i++)
        //             hommingElmo_before[START_N + i] = hommingElmo[START_N + i];
        //     }
            
        //     fz_check = true;
        //     for (int i = 0; i < 18; i++)
        //     {
        //         fz_check = fz_check && (elmofz[fz_group1[i]].result == ElmoHommingStatus::SUCCESS);
        //     }

        //     static bool low_verbose = true;
        //     if (low_verbose && fz_group3_check)
        //     {
        //         printf("ELMO %d : lowerbody zp done \n", init_args->ecat_device);
        //         low_verbose = false;
        //     }

        // if (fz_check)
        //     {
        //         if (saveZeroPoint())
        //         {
        //             // printf("ELMO : zeropoint searching complete, saved \n");
        //             de_zp_sequence = false;
        //             break;
        //         }
        //         else
        //         {
        //             // printf("ELMO : zeropoint searching complete, save failed\n");
        //             de_zp_sequence = false;
        //             break;
        //         }
        //     }
        // }


//rui - /*get joint torque*/
        getJointCommand();
        
        
        // writeDevice();
//rui - /*send command*/
        // if(odrv.areMotorsReady()){
        //     for(int i=0; i< DyrosBoltModel::HW_TOTAL_DOF / 2 - 1; i++)
        //     {
        //         odrv.setInputTorque(i, double(-desired_torque_(i)/k_tau[i]));
        //         odrv.setInputTorque(i+3, double(desired_torque_(i+4)/k_tau[i+3]));
        //     }
        // }

        // for (int i = 0; i < ec_slavecount; i++)
        // {

        //     if (ElmoMode[START_N + i] == EM_POSITION)
        //     {
        //         txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousPositionmode;
        //         txPDO[i]->targetPosition = (int)(elmo_axis_direction[START_N + i] * RAD2CNT[START_N + i] * q_desired_elmo_[START_N + i]);

        //         txPDO[i]->maxTorque = 500;

        //     }
        //     else if (ElmoMode[START_N + i] == EM_TORQUE)
        //     {
        //         txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;

        //         txPDO[i]->targetTorque = (int)(torque_desired_elmo_[START_N + i] * NM2CNT[START_N + i] * elmo_axis_direction[START_N + i]);
        //     }
        //     else if (ElmoMode[START_N + i] == EM_COMMUTATION)
        //     {
        //         txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
        //         txPDO[i]->targetTorque = (int)0;
        //     }
        //     else
        //     {
        //         txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
        //         txPDO[i]->targetTorque = (int)0;
        //     }
        // }
        


    }
    return (void *)NULL;

}
void readDevice(){

    for (int i = 0; i < 3; i++)
    {
        q_[i] = -odrv.axis_angle[i];
        q_dot_[i] = odrv.axis_velocity[i];

        q_[i+3] = odrv.axis_angle[i+3];
        q_dot_[i+3] = odrv.axis_velocity[i+3];
    }
    q_[3] = -q_[3];

    imu_data_quat[0] = shm_msgs_->pos_virtual[0];
    imu_data_quat[1] = shm_msgs_->pos_virtual[1];
    imu_data_quat[2] = shm_msgs_->pos_virtual[2];
    imu_data_quat[3] = shm_msgs_->pos_virtual[3];

    imu_accelometer_[0] = shm_msgs_->imu_acc[0];
    imu_accelometer_[1] = shm_msgs_->imu_acc[1];
    imu_accelometer_[2] = shm_msgs_->imu_acc[2];

    imu_angular_velocity[0] = shm_msgs_->vel_virtual[0];
    imu_angular_velocity[1] = shm_msgs_->vel_virtual[1];
    imu_angular_velocity[2] = shm_msgs_->vel_virtual[2];

    // std::cout << "q_" << std::endl;
    // for (size_t i = 0; i < 6; i++)
    // {
    //     std::cout << q_[i] << std::endl;
        
    // }
    // std::cout << "q_dot_" << std::endl;
    // for (size_t i = 0; i < 6; i++)
    // {
    //     std::cout << q_dot_[i] << std::endl;
    // }

}


void update(){
    // if (!extencoder_init_flag_ && q_ext_.squaredNorm() > 0 && q_.squaredNorm() > 0)
    // {
    //     for (int i=0; i<12; i++)
    //     extencoder_offset_(i) = q_[i]-q_ext_[i];

    //     extencoder_init_flag_ = true;
    // }

    // if(extencoder_init_flag_ == true)
    // {
    //     q_ext_offset_ = q_ext_ + extencoder_offset_;
    // }

}

void writeDevice(){
    if(areMotorsReady()){
        
            for(int i=0; i<  MODEL_DOF / 2; i++)
            {
                odrv.setInputTorque(i, float(-torque_desired_[i]/k_tau[i]));
                odrv.setInputTorque(i+3, float(torque_desired_[i+3]/k_tau[i+3]));
            }

        }


    }
    
double elmoJointMove(double current_time, double init, double angle, double start_time, double traj_time)
{
    double des_pos;

    if (current_time < start_time)
    {
        des_pos = init;
    }
    else if ((current_time >= start_time) && (current_time < (start_time + traj_time)))
    {
        des_pos = init + angle * (current_time - start_time) / traj_time;
    }
    else if (current_time >= (start_time + traj_time))
    {
        des_pos = init + angle;
    }

    return des_pos;
}

void sendJointStatus()
{

    shm_msgs_->statusWriting++;

    memcpy(&shm_msgs_->pos[Q_START], &q_[Q_START], sizeof(float) * PART_CAN_DOF);
    memcpy(&shm_msgs_->vel[Q_START], &q_dot_[Q_START], sizeof(float) * PART_CAN_DOF);
    memcpy(&shm_msgs_->torqueActual[Q_START], &torque_[Q_START], sizeof(float) * PART_CAN_DOF);

    shm_msgs_->statusWriting--;

    if (g_init_args.is_main)
    {
        shm_msgs_->triggerS1 = true;

        cpu_relax();
        // atomic_store(&shm_->triggerS1,1,memory_order_release);
    }

}


void getJointCommand()
{
    timespec ts_us1;

    ts_us1.tv_sec = 0;
    ts_us1.tv_nsec = 1000;
    cpu_relax();
    // static int stloop;
    // static bool stloop_check;
    // stloop_check = false;
    // if (stloop == shm_msgs_->stloopCount)
    // {
    //     stloop_check = true;
    // }
    // stloop = shm_msgs_->stloopCount;
    // static int commandCount;
    // int wait_tick;

    
        while (shm_msgs_->cmd_lower)
        {
            clock_nanosleep(CLOCK_MONOTONIC, 0, &ts_us1, NULL);
        };
        shm_msgs_->cmd_lower = true;
        //ANCHOR - torque comming from
        memcpy(&torque_desired_[Q_START], &shm_msgs_->torqueCommand[Q_START], sizeof(float) * PART_CAN_DOF);
        // for(int i = Q_START; i < Q_START + PART_CAN_DOF; i++) {
        //     std::cout << "torque_desired_[" << i << "] = " << torque_desired_[i] << std::endl;
        // }
        shm_msgs_->cmd_lower = false;



    
    // commandCount = shm_msgs_->commandCount;

    // static int commandCount_before = -1;
    // static int commandCount_before2 = -1;
    // static int errorCount = -2;
    // static int errorTimes = 0;

    // static bool start_observe = false;

    // if (!start_observe)
    // {
    //     if (shm_msgs_->controlModeUpper && shm_msgs_->controlModeUpper)
    //     {
    //         start_observe = true;
    //     }
    // }

    // commandCount_before2 = commandCount_before;
    // commandCount_before = commandCount;

}

int kbhit(void)
{
    struct termios oldt, newt;
    int ch = 0;
    int oldf = 0;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO | ISIG);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    int nread = read(0, &ch, 1);
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (nread >= 1)
    {
        return ch;
    }
    else
    {
        return -1;
    }
}

bool areMotorsReady()
{
    for(int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
        if(odrv.axis_current_state[i] != 8) {
            std::cout << "motor is not ready" << std::endl;
            return false;
        }
    }
    return true;
}