// #include <ros/ros.h>
// #include "dyros_bolt_real_robot/odrive_socketcan.h"
// #include "dyros_bolt_real_robot/control_base.h"
// #include "dyros_bolt_real_robot/real_robot_interface.h"

// #include <iostream>
// #include <math.h>
// #include <signal.h>

// // void SIGINT_handler(int sig)
// // {
// //     shutdownSystem();
// // }

// using namespace dyros_bolt_real_robot;

// // #define stack64k (64 * 1024)
// int main(int argc, char **argv)
// {   
//     // mlockall(MCL_CURRENT | MCL_FUTURE);
//     // signal(SIGINT, SIGINT_handler);

//     // ros::init(argc, argv, "dyros_bolt_real_robot");
//     // ros::NodeHandle nh("~");
//     ControlBase *ctr_obj;


//     // double Hz;
//     // nh.param<double>("control_frequency", Hz, 150.0);

//     ctr_obj = new RealRobotInterface();
    
//     // int shm_id_;
//     // init_shm(shm_msg_key, shm_id_, &ctr_obj->tc_shm_);
//     // prog_shutdown = &ctr_obj->tc_shm_->shutdown;
    
//     // ROS_INFO("TEST2");
//     while(ros::ok())
//     {
//         std::cout << "main_real_robot" << std::endl;
//     // while (!shm_msgs_->shutdown)
//     // {
//         ctr_obj->readDevice();
//         ctr_obj->update();
//         ctr_obj->compute();
//         // ctr_obj->reflect();
//         // ctr_obj->writeDevice();
//         // std::copy(tc_shm_->torqueCommand, tc_shm_->torqueCommand + 6, torque_command_);

//     }    
//     //     // ctr_obj->wait();

//     //     // if(ctr_obj->isShuttingDown())
//     //     // {
//     //     //   break;
//     //     // }
        
//     // }
    

    
//     delete ctr_obj;
//     return 0;
// }
#include "dyros_bolt_real_robot/odrive_socketcan_master.h"

bool initTocabiArgs(const TocabiInitArgs &args)
{

  q_.setZero();
  q_dot_.setZero();
  q_dot_filtered_.setZero();
  torque_.setZero();
  desired_torque_.setZero();
  desired_q_.setZero();
  extencoder_init_flag_ = false;

    init_shm(shm_msg_key, shm_id_, &shm_msgs_);
    odrvInit();

    if (shm_msgs_->shutdown == true)
    {

        printf("ELMO %d : shm reset\n", args.ecat_device);
    }


    return true;
}

bool initDyrosBoltSystem(const TocabiInitArgs &args)
{

    // const char *ifname1 = args.port1.c_str();
    char ifname2[100];
    strcpy(ifname2, args.port2);
    // char *ifname2 = args.port2;

    if (!ec_init_redundant(args.port1, ifname2))
    // if (!ec_init(args.port1))
    {
        fprintf(stdout, "ELMO %d : No socket connection on %s / %s \nExcecute as root\n", args.ecat_device, args.port1, args.port2);
        return false;
    }

    fprintf(stdout, "ELMO %d : ec_init on %s %s succeeded.\n", args.ecat_device, args.port1, args.port2);
    fflush(stdout);
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

    TocabiInitArgs *init_args = (TocabiInitArgs *)data;

    if (ec_config_init(FALSE) <= 0) // TRUE when using configtable to init slavtes, FALSE oherwise
    {
        fprintf(stdout, "%sELMO : No slaves found!%s\n", cred, creset);
    }
    fprintf(stdout, "ELMO %d : %d / %d slaves found and configured.\n", g_init_args.ecat_device, ec_slavecount, g_init_args.ecat_slave_num); // ec_slavecount -> slave num

    if (ec_slavecount == g_init_args.ecat_slave_num)
    {
        ecat_number_ok = true;
    }
    else
    {
        fprintf(stdout, "ELMO %d : %d / %d slaves found and configured.\n", g_init_args.ecat_device, ec_slavecount, g_init_args.ecat_slave_num); // ec_slavecount -> slave num
        shm_msgs_->shutdown = true;
    }
    /** CompleteAccess disabled for Elmo driver */
    for (int slave = 1; slave <= ec_slavecount; slave++)
    {
        if (!(ec_slave[slave].CoEdetails & ECT_COEDET_SDOCA))
        {
            fprintf(stdout, "ELMO %d : slave[%d] CA? : false , shutdown request \n ", g_init_args.ecat_device, slave);
        }
        ec_slave[slave].CoEdetails ^= ECT_COEDET_SDOCA;
    }

    for (int slave = 1; slave <= ec_slavecount; slave++)
    {
        // 0x1605 :  Target Position             32bit 4
        //           Target Velocity             32bit 4
        //           Max Torque                  16bit 2
        //           Control word                16bit 2
        //           Modes of Operation          16bit 2
        uint16 map_1c12[2] = {0x0001, 0x1605};

        // 0x1a00 :  position actual value       32bit 4B
        //           Digital Inputs              32bit 4B
        //           Status word                 16bit 2B
        // 0x1a11 :  velocity actual value       32bit 4B
        // 0x1a13 :  Torque actual value         16bit 2B
        // 0x1a1e :  Auxiliary position value    32bit 4B
        uint16 map_1c13[5] = {0x0004, 0x1a00, 0x1a11, 0x1a13, 0x1a1e}; //, 0x1a12};
        // uint16 map_1c13[6] = {0x0005, 0x1a04, 0x1a11, 0x1a12, 0x1a1e, 0X1a1c};
        int os;
        os = sizeof(map_1c12);
        int r = ec_SDOwrite(slave, 0x1c12, 0, TRUE, os, map_1c12, EC_TIMEOUTRXM);
        if (r < 0)
            printf("%sELMO %d : unable to write sdo map_1c12 to ecat %d%s\n", cred, init_args->ecat_device, slave, creset);
        os = sizeof(map_1c13);
        r = ec_SDOwrite(slave, 0x1c13, 0, TRUE, os, map_1c13, EC_TIMEOUTRXM);
        if (r < 0)
            printf("%sELMO %d : unable to write sdo map_1c13 to ecat %d%s\n", cred, init_args->ecat_device, slave, creset);
    }
    /** if CA disable => automapping works */

    if (init_args->verbose)
        fprintf(stdout, "ELMO %d : EC CONFIG MAP\n", init_args->ecat_device);
    fflush(stdout);

    int ecmap = ec_config_map(&IOmap);

    // printf("ELMO %d : EC CONFIG MAP RES : %d IOmap Size : %d \n", init_args->ecat_device, ecmap, sizeof(IOmap));
    // ec_config_overlap_map(IOmap);

    // ecdc
#ifdef ECAT_DC
    printf("ELMO %d : EC CONFIG DC\n", init_args.ecat_device);
    ec_configdc();

    for (int i = 0; i < ec_slavecount; i++)
        ec_dcsync0(i + 1, TRUE, (uint32)init_args.period_ns, 0);

#endif
    while (EcatError)
        printf("%s", ec_elist2string());

    if (init_args->verbose)
        fprintf(stdout, "ELMO %d : EC WAITING STATE TO SAFE_OP\n", init_args->ecat_device);
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    ec_readstate();
    /* wait for all slaves to reach SAFE_OP state */
    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    if (init_args->verbose)
        fprintf(stdout, "ELMO %d : Request operational state for all slaves. Calculated workcounter : %d\n", init_args->ecat_device, expectedWKC);

    if (expectedWKC != 3 * init_args->ecat_slave_num)
    {
        // std::cout << cred << "WARNING : Calculated Workcounter insufficient!" << creset << '\n';
        ecat_WKC_ok = true;
    }

    /** going operational */
    ec_slave[0].state = EC_STATE_OPERATIONAL;

    /* send one valid process data to make outputs in slaves happy*/
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    /* request OP state for all slaves */
    ec_writestate(0);

    int wait_cnt = 40;

    int64 toff, gl_delta;
    toff = 0;
    unsigned long long cur_dc32 = 0;
    unsigned long long pre_dc32 = 0;
    long long diff_dc32 = 0;
    long long cur_DCtime = 0, max_DCtime = 0;

    /* wait for all slaves to reach OP state */
    do
    {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 5000);
    } while (wait_cnt-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

    if (ec_slave[0].state != EC_STATE_OPERATIONAL)
    {
        fprintf(stdout, "%sELMO %d : Not all slaves reached operational state.%s\n", cred, init_args->ecat_device, creset);
        ec_readstate();
        for (int slave = 1; slave <= ec_slavecount; slave++)
        {
            if (ec_slave[slave - 1].state != EC_STATE_OPERATIONAL)
            {
                fprintf(stdout, "%sELMO %d : EtherCAT State Operation Error : Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s%s\n",
                        cred, init_args->ecat_device, slave - 1, ec_slave[slave - 1].state, ec_slave[slave - 1].ALstatuscode,
                        ec_ALstatuscode2string(ec_slave[slave - 1].ALstatuscode), creset);
            }
        }
        shm_msgs_->shutdown = true;
    }
    for (int slave = 1; slave <= ec_slavecount; slave++)
    {
        txPDO[slave - 1] = (EtherCAT_Elmo::ElmoGoldDevice::elmo_gold_tx *)(ec_slave[slave].outputs);
        rxPDO[slave - 1] = (EtherCAT_Elmo::ElmoGoldDevice::elmo_gold_rx *)(ec_slave[slave].inputs);
    }

    inOP = TRUE;
    const int PRNS = init_args->period_ns;
    period_ns = init_args->period_ns;

    // //ecdc
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
#ifdef ECAT_DC
    for (int i = 0; i < ec_slavecount; i++)
        ec_dcsync0(i + 1, TRUE, (uint32)args.period_ns, 2000);
    toff = 0;

    ec_send_processdata();

    wkc = ec_receive_processdata(EC_TIMEOUTRET);

    cur_dc32 = (uint32_t)(ec_DCtime & 0xffffffff);

    long dc_remain_time = cur_dc32 % PRNS;
    ts.tv_nsec = ts.tv_nsec - ts.tv_nsec % PRNS + dc_remain_time;
    while (ts.tv_nsec >= SEC_IN_NSEC)
    {
        ts.tv_sec++;
        ts.tv_nsec -= SEC_IN_NSEC;
    }
#endif

    // std::printf("dc_remain_time : " << dc_remain_time << '\n';

    // clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);

    /* cyclic loop */

    // Commutation Checking
    //  st_start_time = std::chrono::steady_clock::now(); // TODO:timespec
    fprintf(stdout, "%sELMO %d : START Initialization Mode %s\n", cyellow, g_init_args.ecat_device, creset);
    fflush(stdout);

    if (g_init_args.ecat_device == 1)
    {
        shm_msgs_->initializeModeUpper = true;
    }
    else if (g_init_args.ecat_device == 2)
    {
        shm_msgs_->initializeModeLower = true;
    }

    query_check_state = true;
    g_toff = toff;
    g_cur_dc32 = cur_dc32;
    g_pre_dc32 = pre_dc32;
    g_diff_dc32 = diff_dc32;
    g_cur_DCtime = cur_DCtime;
    g_PRNS = PRNS;
    g_ts = ts;

    // int64 toff = 0;
    // unsigned long long cur_dc32 = g_cur_dc32;
    // unsigned long long pre_dc32 = g_pre_dc32;
    // long long diff_dc32 = g_diff_dc32;
    // long long cur_DCtime = g_cur_dc32, max_DCtime = g_max_DCtime;
    // int PRNS = g_PRNS;
    // struct timespec ts;

    struct timespec ts_now;
    bool reachedInitial[ELMO_DOF] = {false};
    // force_control_mode = true;
    de_debug_level++;
    status_log = true;

    struct timespec ts_start;
    clock_gettime(CLOCK_MONOTONIC, &ts_start); // ADDED
    clock_gettime(CLOCK_MONOTONIC, &ts);       // ADDED

    struct timespec ts_tt;
    char buff[100];

    timespec_get(&ts_tt, TIME_UTC);
    ts_tt.tv_sec += 60 * 60 * 9;
    strftime(buff, sizeof buff, "%D %T", gmtime(&ts_tt.tv_sec));

    if (init_args->ecat_device == 0)
        printf("ELMO %d : entering initialize START_N %d jointNUM %d QSTART %d | %s\n", init_args->ecat_device, START_N, init_args->ecat_slave_num, init_args->q_start_, buff);

    if (shm_msgs_->shutdown)
        printf("Shutdown Command Before Start\n");

    while (!shm_msgs_->shutdown)
    {
        if (force_control_mode)
        {
            break;
        }

        ts.tv_nsec += init_args->period_ns + toff;
        if (ts.tv_nsec >= SEC_IN_NSEC)
        {
            ts.tv_sec++;
            ts.tv_nsec -= SEC_IN_NSEC;
        }
        ec_send_processdata();
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);

        // std::this_thread::sleep_until(st_start_time + cycle_count * cycletime);
        cycle_count++;

        // printf("ELMO %d : entering initialize START_N %d jointNUM %d\n", init_args->ecat_device, START_N, init_args->ecat_slave_num);

        // if ((cycle_count % 2000) == 0)
        //     printf("ELMO %d running \n", g_init_args.ecat_device);
        wkc = ec_receive_processdata(EC_PACKET_TIMEOUT);

        control_time_real_ = cycle_count * init_args->period_ns / 1000000000.0;
        // if (init_args->is_main)
        //     shm_msgs_->control_time_real_ = control_time_real_;
        // control_time_real_ = std::chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - st_start_time).count() / 1000000.0;
        // while (EcatError)
        //     printf("%f %s", control_time_real_, ec_elist2string());

        for (int i = 0; i < ec_slavecount; i++)
        {
            // printf("%d\t",rxPDO[i]->statusWord);

            elmost[i].state = getElmoState(rxPDO[i]->statusWord);

            if (elmost[i].state != elmost[i].state_before)
            {
                state_elmo_[JointMap2[START_N + i]] = elmost[i].state;

                if (elmost[i].first_check)
                {
                    if (elmost[i].state == ELMO_NOTFAULT)
                    {
                        elmost[i].commutation_required = true;
                    }
                    else if (elmost[i].state == ELMO_FAULT)
                    {
                        // printf("slave : " << i << " commutation check complete at first\n");
                        elmost[i].commutation_not_required = true;
                    }
                    else if (elmost[i].state == ELMO_OPERATION_ENABLE)
                    {
                        // printf("slave : " << i << " commutation check complete with operation enable\n");
                        elmost[i].commutation_not_required = true;
                        elmost[i].commutation_ok = true;
                    }
                    else
                    {
                        // printf("first missing : slave : " << i << " state : " << elmost[i].state << '\n';
                    }
                    elmost[i].first_check = false;
                }
                else
                {
                    if (elmost[i].state == ELMO_OPERATION_ENABLE)
                    {
                        // printf("slave : %d commutation check complete with operation enable 2\n",i);
                        elmost[i].commutation_ok = true;
                        elmost[i].commutation_required = false;
                    }
                }
                query_check_state = true;
            }
            elmost[i].state_before = elmost[i].state;
        }
        // printf("\n");

        if (check_commutation)
        {
            if (check_commutation_first)
            {
                if (init_args->ecat_device == 0)
                {
                    printf("Commutation Status : \n");
                    for (int i = 0; i < ec_slavecount; i++)
                        printf("--");
                    printf("\n");
                    for (int i = 0; i < ec_slavecount; i++)
                        printf("%2d", (i - i % 10) / 10);
                    printf("\n");
                    for (int i = 0; i < ec_slavecount; i++)
                        printf("%2d", i % 10);
                    printf("\n");
                    printf("\n");
                    printf("\n");
                }
                check_commutation_first = false;
            }
            if (query_check_state)
            {
                if (init_args->ecat_device == 0)
                {
                    printf("\x1b[A\x1b[A\33[2K\r");
                    for (int i = 0; i < ec_slavecount; i++)
                    {
                        if (elmost[i].state == ELMO_OPERATION_ENABLE)
                        {
                            printf("%s%2d%s", cgreen, elmost[i].state, creset);
                        }
                        else
                        {
                            printf("%2d", elmost[i].state);
                        }
                    }
                    printf("\n");
                    for (int i = 0; i < ec_slavecount; i++)
                        printf("--");
                    printf("\n");
                    fflush(stdout);
                }
                query_check_state = false;
            }
        }

        bool waitop = true;
        for (int i = 0; i < ec_slavecount; i++)
            waitop = waitop && elmost[i].commutation_ok;

        if (waitop)
        {
            static bool pub_once = true;

            if (pub_once)
            {

                // std::chrono::milliseconds commutation_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - st_start_time);
                long commutation_time = getTimeDiff(ts_start);

                int commutation_min_time = 1000;

                if (commutation_time / 1e6 < commutation_min_time)
                {

                    de_commutation_done = true;
                    check_commutation = false;
                    if (init_args->verbose)
                        printf("ELMO %d : Load ZP ... \n", init_args->ecat_device);
                }
                else
                {

                    printf("ELMO %d : All slaves Operational in %f ms, > %d\n", init_args->ecat_device, commutation_time / 1e6, commutation_min_time);

                    if (saveCommutationLog())
                    {
                        if (init_args->verbose)
                            printf("\nELMO %d : Commutation is done, logging success\n", init_args->ecat_device);
                    }
                    else
                    {
                        printf("\nELMO %d : Commutation is done, logging failed\n", init_args->ecat_device);
                    }
                    de_commutation_done = true;
                    check_commutation = false;
                }

                pub_once = false;
            }
        }

        if (de_commutation_done)
        {
            static bool pub_once = true;
            if (pub_once)
            {

                if (loadZeroPoint())
                {
                    // if (init_args->verbose)
                    //     printf("ELMO %d : Initialize Complete \n", init_args->ecat_device);
                    break;
                }
                else
                {
                    printf("ELMO %d : ZeroPoint load failed. Ready to Search Zero Point \n", init_args->ecat_device);
                    de_zp_sequence = true;
                }
                pub_once = false;
            }
        }

        if (shm_msgs_->force_load_saved_signal)
        {
            loadZeroPoint(true);
            printf("ELMO 1 : force load ZP\n");
            break;
        }

        bool waitcm = true;
        for (int i = 0; i < ec_slavecount; i++)
            waitcm = waitcm && elmost[i].commutation_not_required;

        if (waitcm)
        {
            if (wait_kill_switch)
            {
                if (init_args->verbose)
                    printf("ELMO %d : Commutation state OK\n", init_args->ecat_device);
                // loadCommutationLog();
                loadZeroPoint();
                wait_kill_switch = false;
                check_commutation = false;
            }
            if (wait_cnt == 200)
            {
                printf("ELMO %d : slaves status are not OP! maybe kill switch is on?\n", init_args->ecat_device);
            }

            wait_cnt++;
        }
        else
        {
            int total_commutation_cnt = 0;
            for (int i = 0; i < ec_slavecount; i++)
            {
                if (elmost[i].commutation_required)
                {
                    total_commutation_cnt++;
                    if (total_commutation_cnt < 4)
                        controlWordGenerate(rxPDO[i]->statusWord, txPDO[i]->controlWord);
                    txPDO[i]->maxTorque = (uint16)400; // originaly 1000
                }
            }
        }

        if (wkc >= expectedWKC)
        {
            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                if (!elmost[slave - 1].commutation_required)
                {
                    if (controlWordGenerate(rxPDO[slave - 1]->statusWord, txPDO[slave - 1]->controlWord))
                    {
                        reachedInitial[slave - 1] = true;
                    }
                    if (reachedInitial[slave - 1])
                    {
                        q_elmo_[START_N + slave - 1] = rxPDO[slave - 1]->positionActualValue * CNT2RAD[START_N + slave - 1] * elmo_axis_direction[START_N + slave - 1]; // - q_zero_elmo_[START_N + slave - 1];

                        if (START_N + slave - 1 == R_HipYaw_Joint)
                        {
                            grav_signal = (((uint32_t)ec_slave[slave].inputs[6]) & ((uint32_t)1));
                            pos_signal = (((uint32_t)ec_slave[slave].inputs[6]) & ((uint32_t)2));
                        }

                        hommingElmo[START_N + slave - 1] =
                            (((uint32_t)ec_slave[slave].inputs[6]) & ((uint32_t)1));
                        q_dot_elmo_[START_N + slave - 1] =
                            (((int32_t)ec_slave[slave].inputs[10]) +
                             ((int32_t)ec_slave[slave].inputs[11] << 8) +
                             ((int32_t)ec_slave[slave].inputs[12] << 16) +
                             ((int32_t)ec_slave[slave].inputs[13] << 24)) *
                            CNT2RAD[START_N + slave - 1] * elmo_axis_direction[START_N + slave - 1];
                        torque_elmo_[START_N + slave - 1] =
                            (int16_t)(((int16_t)ec_slave[slave].inputs[14]) +
                                      ((int16_t)ec_slave[slave].inputs[15] << 8));
                        q_ext_elmo_[START_N + slave - 1] =
                            (((int32_t)ec_slave[slave].inputs[16]) +
                             ((int32_t)ec_slave[slave].inputs[17] << 8) +
                             ((int32_t)ec_slave[slave].inputs[18] << 16) +
                             ((int32_t)ec_slave[slave].inputs[19] << 24) - q_ext_mod_elmo_[START_N + slave - 1]) *
                            EXTCNT2RAD[START_N + slave - 1] * elmo_ext_axis_direction[START_N + slave - 1];

                        if (q_ext_elmo_[START_N + slave - 1] > 3.141592)
                        {
                            q_ext_elmo_[START_N + slave - 1] -= 3.141592 * 2;
                        }
                        else if (q_ext_elmo_[START_N + slave - 1] < -3.141592)
                        {
                            q_ext_elmo_[START_N + slave - 1] += 3.141592 * 2;
                        }

                        // st_register[START_N + slave - 1] =
                        //     ((uint32_t)ec_slave[slave].inputs[20]) +
                        //      ((uint32_t)ec_slave[slave].inputs[21] << 8) +
                        //      ((uint32_t)ec_slave[slave].inputs[22] << 16) +
                        //      ((uint32_t)ec_slave[slave].inputs[23] << 24);
                        if (START_N + slave == 1 || START_N + slave == 2 || START_N + slave == 7 || START_N + slave == 19 || START_N + slave == 20 || START_N + slave == 16)
                        {
                            hommingElmo[START_N + slave - 1] = !hommingElmo[START_N + slave - 1];
                        }
                        txPDO[slave - 1]->maxTorque = (uint16)500; // originaly 1000
                    }
                }
            }
        }

        // for (int slave = 1; slave <= ec_slavecount; slave++)
        // {
        //     // if (!elmost[slave - 1].commutation_required)
        //     // {
        //         if (controlWordGenerate(rxPDO[slave - 1]->statusWord, txPDO[slave - 1]->controlWord))
        //         {
        //             reachedInitial[slave - 1] = true;
        //         }
        //     // }

        //     if (reachedInitial[slave - 1])
        //     {
        //         q_elmo_[START_N + slave - 1] = rxPDO[slave - 1]->positionActualValue * CNT2RAD[START_N + slave - 1] * elmo_axis_direction[START_N + slave - 1];
        //         hommingElmo[START_N + slave - 1] =
        //             (((uint32_t)ec_slave[slave].inputs[6]) & ((uint32_t)1));
        //         q_dot_elmo_[START_N + slave - 1] =
        //             (((int32_t)ec_slave[slave].inputs[10]) +
        //              ((int32_t)ec_slave[slave].inputs[11] << 8) +
        //              ((int32_t)ec_slave[slave].inputs[12] << 16) +
        //              ((int32_t)ec_slave[slave].inputs[13] << 24)) *
        //             CNT2RAD[START_N + slave - 1] * elmo_axis_direction[START_N + slave - 1];
        //         torque_elmo_[START_N + slave - 1] =
        //             (int16_t)(((int16_t)ec_slave[slave].inputs[14]) +
        //                       ((int16_t)ec_slave[slave].inputs[15] << 8));
        //         q_ext_elmo_[START_N + slave - 1] =
        //             (((int32_t)ec_slave[slave].inputs[16]) +
        //              ((int32_t)ec_slave[slave].inputs[17] << 8) +
        //              ((int32_t)ec_slave[slave].inputs[18] << 16) +
        //              ((int32_t)ec_slave[slave].inputs[19] << 24) - q_ext_mod_elmo_[START_N + slave - 1]) *
        //             EXTCNT2RAD[START_N + slave - 1] * elmo_ext_axis_direction[START_N + slave - 1];
        //         // st_register[START_N + slave - 1] =
        //         //     ((uint32_t)ec_slave[slave].inputs[20]) +
        //         //      ((uint32_t)ec_slave[slave].inputs[21] << 8) +
        //         //      ((uint32_t)ec_slave[slave].inputs[22] << 16) +
        //         //      ((uint32_t)ec_slave[slave].inputs[23] << 24);

        //         if (START_N + slave == 1 || START_N + slave == 2 || START_N + slave == 7 || START_N + slave == 19 || START_N + slave == 20 || START_N + slave == 16)
        //         {
        //             hommingElmo[START_N + slave - 1] = !hommingElmo[START_N + slave - 1];
        //         }
        //         txPDO[slave - 1]->maxTorque = (uint16)100; // originaly 1000
        //     }
        // }

        for (int i = 0; i < ec_slavecount; i++)
        {
            q_[JointMap2[START_N + i]] = q_elmo_[START_N + i];
            q_dot_[JointMap2[START_N + i]] = q_dot_elmo_[START_N + i];
            torque_[JointMap2[START_N + i]] = torque_elmo_[START_N + i];

            if (g_init_args.ecat_device == 1)
                q_ext_[JointMap2[START_N + i]] = q_desired_elmo_[START_N + i];
            else
                q_ext_[JointMap2[START_N + i]] = q_ext_elmo_[START_N + i];

            // joint_state_[JointMap2[START_N + i]] = joint_state_elmo_[START_N + i];
        }

        // for(int i=0;i<ec_slavecount;i++)
        // {
        //     shm_msgs_->pos[]
        // }

        // shm_msgs_->pos[JointMap2[START_N + i]] = q_elmo_[START_N + i];
        if (g_init_args.ecat_device == 1)
        {
            if (shm_msgs_->upper_init_signal)
            {
                de_zp_upper_switch = true;
                shm_msgs_->upper_init_signal = false;
            }
        }
        if (g_init_args.ecat_device == 2)
        {

            if (shm_msgs_->low_init_signal)
            {
                de_zp_lower_switch = true;
                shm_msgs_->low_init_signal = false;
            }
            if (shm_msgs_->waist_init_signal)
            {
                de_zp_upper_switch = true;
                shm_msgs_->waist_init_signal = false;
            }
        }

        sendJointStatus();
        if (de_zp_sequence)
        {
            static bool zp_upper = false;
            static bool zp_lower = false;

            if (de_zp_upper_switch)
            {
                printf("ELMO %d : Starting upper zp\n", init_args->ecat_device);
                // for (int i = 0; i < 8; i++)
                //     printf("L" << i << "\t";
                // for (int i = 0; i < 8; i++)
                //     printf("R" << i << "\t";
                // cout << '\n';
                elmofz[R_Shoulder3_Joint].findZeroSequence = 7;
                elmofz[R_Shoulder3_Joint].initTime = control_time_real_;
                elmofz[L_Shoulder3_Joint].findZeroSequence = 7;
                elmofz[L_Shoulder3_Joint].initTime = control_time_real_;

                for (int i = 0; i < ec_slavecount; i++)
                    hommingElmo_before[START_N + i] = hommingElmo[START_N + i];

                de_zp_upper = true;
                de_zp_upper_switch = false;
            }

            if (de_zp_lower_switch)
            {
                printf("ELMO %d : Starting lower zp\n", init_args->ecat_device);
                de_zp_lower_switch = false;
                zp_lower = true;
            }

            if (de_zp_upper)
            {
                if (init_args->ecat_device == 1) // Upper
                {

                    for (int i = 0; i < 18; i++)
                    {
                        findZeroPoint(fz_group1[i], control_time_real_);
                    }
                }
                else // lower
                {
                    for (int i = 0; i < 3; i++)
                    {
                        findZeroPoint(fz_group2[i], control_time_real_);
                    }
                }
                for (int i = 0; i < ec_slavecount; i++)
                    hommingElmo_before[START_N + i] = hommingElmo[START_N + i];
            }

            if (zp_lower)
            {
                if (zp_lower_calc)
                {
                    findzeroLeg();
                    zp_lower_calc = false;
                }
                else
                {
                    for (int i = 0; i < 6; i++)
                    {
                        findZeroPointlow(i + R_HipYaw_Joint, control_time_real_);
                        findZeroPointlow(i + L_HipYaw_Joint, control_time_real_);
                    }
                }
            }

            if (g_init_args.ecat_device == 1)
            {
                fz_group1_check = true;
                for (int i = 0; i < 18; i++)
                {
                    fz_group1_check = fz_group1_check && (elmofz[fz_group1[i]].result == ElmoHommingStatus::SUCCESS);
                }
            }

            if (g_init_args.ecat_device == 2)
            {
                fz_group2_check = true;
                for (int i = 0; i < 3; i++)
                {
                    fz_group2_check = fz_group2_check && (elmofz[fz_group2[i]].result == ElmoHommingStatus::SUCCESS);
                }
                fz_group3_check = true;

                for (int i = 0; i < 12; i++)
                {
                    fz_group3_check = fz_group3_check && (elmofz[fz_group3[i]].result == ElmoHommingStatus::SUCCESS);
                }
            }

            // if (fz_group1_check && (fz_group == 0))
            // {
            //     // printf("ELMO : arm zp done \n");
            //     // fz_group++;
            // }
            // if (fz_group2_check && (fz_group == 1))
            // {
            //     // fz_group++;
            //     // printf("ELMO : waist zp done\n");
            // }

            static bool low_verbose = true;
            if (low_verbose && fz_group3_check)
            {
                printf("ELMO %d : lowerbody zp done \n", init_args->ecat_device);
                low_verbose = false;
            }
            if (fz_group2_check && g_init_args.ecat_device == 2)
            {
                if (saveZeroPoint())
                {
                    // printf("ELMO : zeropoint searching complete, saved \n");
                    de_zp_sequence = false;
                    break;
                }
                else
                {
                    // printf("ELMO : zeropoint searching complete, save failed\n");
                    de_zp_sequence = false;
                    break;
                }
            }

            if (fz_group1_check && g_init_args.ecat_device == 1)
            {
                if (saveZeroPoint())
                {
                    // printf("ELMO : zeropoint searching complete, saved \n");
                    de_zp_sequence = false;
                    break;
                }
                else
                {
                    // printf("ELMO : zeropoint searching complete, save failed\n");
                    de_zp_sequence = false;
                    break;
                }
            }
        }

        // command
        for (int i = 0; i < ec_slavecount; i++)
        {

            if (ElmoMode[START_N + i] == EM_POSITION)
            {
                txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousPositionmode;
                txPDO[i]->targetPosition = (int)(elmo_axis_direction[START_N + i] * RAD2CNT[START_N + i] * q_desired_elmo_[START_N + i]);

                txPDO[i]->maxTorque = 500;

                // if (START_N + i == Head_Joint)
                // {
                //     printf("qdes : %6.3f q: %6.3f homming: %d pact %d \n", q_desired_elmo_[START_N + i], q_elmo_[START_N + i], hommingElmo[START_N + i], rxPDO[i]->positionActualValue);
                // }
            }
            else if (ElmoMode[START_N + i] == EM_TORQUE)
            {
                txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;

                txPDO[i]->targetTorque = (int)(torque_desired_elmo_[START_N + i] * NM2CNT[START_N + i] * elmo_axis_direction[START_N + i]);
            }
            else if (ElmoMode[START_N + i] == EM_COMMUTATION)
            {
                txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
                txPDO[i]->targetTorque = (int)0;
            }
            else
            {
                txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
                txPDO[i]->targetTorque = (int)0;
            }
        }

#ifdef ECAT_DC
        cur_dc32 = (uint32_t)(ec_DCtime & 0xffffffff);
        if (cur_dc32 > pre_dc32)
            diff_dc32 = cur_dc32 - pre_dc32;
        else
            diff_dc32 = (0xffffffff - pre_dc32) + cur_dc32;
        pre_dc32 = cur_dc32;

        cur_DCtime += diff_dc32;

        ec_sync(cur_DCtime, period_ns, toff);

        if (cur_DCtime > max_DCtime)
            max_DCtime = cur_DCtime;
#endif
    }

    if (!shm_msgs_->shutdown)
        fprintf(stdout, "%sELMO %d : Control Mode Start ... at%ld %s\n", cgreen, g_init_args.ecat_device, cycle_count, creset);
    fflush(stdout);
    // memset(joint_state_elmo_, ESTATE::OPERATION_READY, sizeof(int) * ec_slavecount);
    // st_start_time = std::chrono::steady_clock::now();
    // cycle_count = 1;
    ////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////
    // Starting
    ///////////////////////////////////////////////////////////////////////////////////////////

    int64_t total1, total2, total3;
    int64_t total_dev1, total_dev2;
    int lat_ns;
    int sat_ns;
    int rsat_ns;
    int rat_ns;
    float lmax, lamax, lmin, ldev, lavg, lat;
    float smax, samax, smin, sdev, savg, sat;
    float rmax, ramax, rmin, rdev, ravg, rat;

    rcv_cnt = 0;

    int l_ovf = 0;
    int s_ovf = 0;

    total1 = 0;
    total2 = 0;
    total_dev1 = 0;
    total_dev2 = 0;

    ldev = 0.0;
    lavg = 0.0;
    lat = 0;

    lmax = 0.0;
    lmin = 10000.00;
    smax = 0.0;
    smin = 100000.0;
    rmax = 0.0;

    sdev = 0;
    savg = 0;
    sat = 0;
    rat = 0;

    lamax = 0.0;
    samax = 0.0;
    ramax = 0.0;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    ts.tv_nsec += PRNS;
    while (ts.tv_nsec >= SEC_IN_NSEC)
    {
        ts.tv_sec++;
        ts.tv_nsec -= SEC_IN_NSEC;
    }
    struct timespec ts0;
    struct timespec ts1;
    struct timespec ts12;
    struct timespec ts2;

    uint16_t statusWord[ELMO_DOF];
    uint16_t statusWord_before[ELMO_DOF];
    bool status_first = true;
    control_mode = true;

    int r_us, s_us, l_us;
    int control_cnt = 0;

    if (init_args->ecat_device == 1)
    {
        shm_msgs_->controlModeUpper = true;
    }
    else if (init_args->ecat_device == 2)
    {
        shm_msgs_->controlModeLower = true;
    }

    while (!shm_msgs_->shutdown)
    {
        int mask = 0;

        if (g_init_args.ecat_device == 1) // mask 0
        {
            shm_msgs_->e1_m[mask++]++;
        }
        else if (g_init_args.ecat_device == 2)
        {
            shm_msgs_->e2_m[mask++]++;
        }

        bool latency_no_count = false;

        control_cnt++;

        clock_gettime(CLOCK_MONOTONIC, &ts1);

        // if (getTimeDiff(ts1, ts) < 0)
        // {
        //     latency_no_count = true;
        //     int ov_cnt = 0;
        //     while (getTimeDiff(ts1, ts) < 0)
        //     {
        //         ts.tv_nsec += PRNS;
        //         while (ts.tv_nsec >= SEC_IN_NSEC)
        //         {
        //             ts.tv_sec++;
        //             ts.tv_nsec -= SEC_IN_NSEC;
        //         }
        //         ov_cnt++;
        //     }
        //     printf("ov_cnt event at ELMO %d\n", init_args->ecat_device);
        // }

        clock_gettime(CLOCK_MONOTONIC, &ts0);
        ec_send_processdata();

        clock_gettime(CLOCK_MONOTONIC, &ts1);
        rat_ns = (ts1.tv_sec - ts0.tv_sec) * SEC_IN_NSEC + ts1.tv_nsec - ts0.tv_nsec;

        if (g_init_args.ecat_device == 1) // mask 1
        {
            shm_msgs_->e1_m[mask++]++;
        }
        else if (g_init_args.ecat_device == 2)
        {
            shm_msgs_->e2_m[mask++]++;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        if (g_init_args.ecat_device == 1) // mask 2
        {
            shm_msgs_->e1_m[mask++]++;
        }
        else if (g_init_args.ecat_device == 2)
        {
            shm_msgs_->e2_m[mask++]++;
        }
        control_time_real_ = cycle_count * init_args->period_ns / 1000000000.0;

        clock_gettime(CLOCK_MONOTONIC, &ts1);

        lat_ns = (ts1.tv_sec - ts.tv_sec) * SEC_IN_NSEC + ts1.tv_nsec - ts.tv_nsec;

        if (latency_no_count)
            lat_ns = 0;

        /** PDO I/O refresh */

        clock_gettime(CLOCK_MONOTONIC, &ts1);

        if (control_cnt < 2000)
        {
            if (pthread_mutex_trylock(&rcv_mtx_) == 0)
            {
                pthread_cond_signal(&rcv_cond_);
                pthread_mutex_unlock(&rcv_mtx_);
            }
        }

        clock_gettime(CLOCK_MONOTONIC, &ts12);

        if (g_init_args.ecat_device == 1) // mask 3
        {
            shm_msgs_->e1_m[mask++]++;
        }
        else if (g_init_args.ecat_device == 2)
        {
            shm_msgs_->e2_m[mask++]++;
        }
        wkc = ec_receive_processdata(EC_PACKET_TIMEOUT);
        rcv_cnt++;
        if (g_init_args.ecat_device == 1) // mask4
        {
            shm_msgs_->e1_m[mask++]++;
        }
        else if (g_init_args.ecat_device == 2)
        {
            shm_msgs_->e2_m[mask++]++;
        }

        clock_gettime(CLOCK_MONOTONIC, &ts2);
        sat_ns = (ts2.tv_sec - ts1.tv_sec) * SEC_IN_NSEC + ts2.tv_nsec - ts1.tv_nsec;
        rsat_ns = (ts2.tv_sec - ts12.tv_sec) * SEC_IN_NSEC + ts2.tv_nsec - ts12.tv_nsec;

        ///
        cpu_relax();

        for (int i = 0; i < ec_slavecount; i++)
        {
            statusWord[i] = rxPDO[i]->statusWord;
        }

        if (status_first)
        {
            for (int i = 0; i < ec_slavecount; i++)
                statusWord_before[i] = statusWord[i];
            status_first = false;
        }

        static int status_changed_count = -1;
        if (status_log)
        {
            bool stword_changed = false;
            bool stword_slvs[ELMO_DOF] = {
                0,
            };

            for (int i = 0; i < ec_slavecount; i++)
            {
                elmost[i].state = getElmoState(statusWord[i]);

                if (elmost[i].state_before != elmost[i].state)
                {
                    state_elmo_[JointMap2[START_N + i]] = elmost[i].state;
                }
                elmost[i].state_before = elmost[i].state;

                if (statusWord_before[i] != statusWord[i])
                {
                    stword_changed = true;
                    stword_slvs[i] = true;
                }
            }
            // if (stword_changed)
            // {
            //     for (int i=0; i<ec_slavecount; i++)
            //         printStatusword(statusWord[i]);
            //     printf("\n");
            // }
            // if (stword_changed)
            // {
            //     timespec_get(&ts_tt, TIME_UTC + 9);
            //     strftime(buff, sizeof buff, "%T", gmtime(&ts_tt.tv_sec));
            //     printf("ELMO %d | CNT %ld : PDS EVENT %s.%09ld UTC\n", g_init_args.ecat_device, cycle_count, buff, ts_tt.tv_nsec);
            // }

            // if (status_changed)
            // {
            //     status_changed_count++;
            //     printf("status changed time: %lf\n[bits]\n", control_time_real_);
            //     for (int i = 0; i < ec_slavecount; i++)
            //     {
            //         printf("0x%x\t", rxPDO[i]->statusWord);
            //     }
            //     printf("\n");
            // }
        }

        if (wkc >= expectedWKC)
        {
            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                // checkFault(rxPDO[slave - 1]->statusWord, slave);
                if (controlWordGenerate(rxPDO[slave - 1]->statusWord, txPDO[slave - 1]->controlWord))
                {
                    reachedInitial[slave - 1] = true;
                }
                if (reachedInitial[slave - 1])
                {
                    q_elmo_[START_N + slave - 1] = rxPDO[slave - 1]->positionActualValue * CNT2RAD[START_N + slave - 1] * elmo_axis_direction[START_N + slave - 1] - q_zero_elmo_[START_N + slave - 1];

                    if (START_N + slave - 1 == R_HipYaw_Joint)
                    {
                        grav_signal = (((uint32_t)ec_slave[slave].inputs[6]) & ((uint32_t)1));
                        pos_signal = (((uint32_t)ec_slave[slave].inputs[6]) & ((uint32_t)2));

                        shm_msgs_->grav_signal = grav_signal;
                        shm_msgs_->pos_signal = pos_signal;
                    }

                    hommingElmo[START_N + slave - 1] =
                        (((uint32_t)ec_slave[slave].inputs[6]) & ((uint32_t)1));
                    q_dot_elmo_[START_N + slave - 1] =
                        (((int32_t)ec_slave[slave].inputs[10]) +
                         ((int32_t)ec_slave[slave].inputs[11] << 8) +
                         ((int32_t)ec_slave[slave].inputs[12] << 16) +
                         ((int32_t)ec_slave[slave].inputs[13] << 24)) *
                        CNT2RAD[START_N + slave - 1] * elmo_axis_direction[START_N + slave - 1];
                    torque_elmo_[START_N + slave - 1] =
                        (int16_t)(((int16_t)ec_slave[slave].inputs[14]) +
                                  ((int16_t)ec_slave[slave].inputs[15] << 8)) *
                        elmo_axis_direction[START_N + slave - 1] / NM2CNT[START_N + slave - 1];
                    q_ext_elmo_[START_N + slave - 1] =
                        (((int32_t)ec_slave[slave].inputs[16]) +
                         ((int32_t)ec_slave[slave].inputs[17] << 8) +
                         ((int32_t)ec_slave[slave].inputs[18] << 16) +
                         ((int32_t)ec_slave[slave].inputs[19] << 24) - q_ext_mod_elmo_[START_N + slave - 1]) *
                        EXTCNT2RAD[START_N + slave - 1] * elmo_ext_axis_direction[START_N + slave - 1];

                    if (q_ext_elmo_[START_N + slave - 1] > 3.141592)
                    {
                        q_ext_elmo_[START_N + slave - 1] -= 3.141592 * 2;
                    }
                    else if (q_ext_elmo_[START_N + slave - 1] < -3.141592)
                    {
                        q_ext_elmo_[START_N + slave - 1] += 3.141592 * 2;
                    }
                    // st_register[START_N + slave - 1] =
                    //     ((uint32_t)ec_slave[slave].inputs[20]) +
                    //      ((uint32_t)ec_slave[slave].inputs[21] << 8) +
                    //      ((uint32_t)ec_slave[slave].inputs[22] << 16) +
                    //      ((uint32_t)ec_slave[slave].inputs[23] << 24);
                    if (START_N + slave == 1 || START_N + slave == 2 || START_N + slave == 7 || START_N + slave == 19 || START_N + slave == 20 || START_N + slave == 16)
                    {
                        hommingElmo[START_N + slave - 1] = !hommingElmo[START_N + slave - 1];
                    }
                    txPDO[slave - 1]->maxTorque = (uint16)1; // originaly 1000
                }
            }

            // static int grav_sig_before = 0;
            // static int pos_sig_before = 0;

            // if (grav_sig_before != grav_signal)
            // {
            //     if (grav_signal)
            //         printf("GRAV SIGNAL ON\n");
            //     else
            //     {
            //         printf("GRAV SIGNAL OFF\n");
            //     }
            // }

            // if (pos_sig_before != pos_signal)
            // {
            //     if (pos_signal)
            //         printf("POS SIG ON\n");
            //     else
            //     {
            //         printf("POS SIG OFF\n");
            //     }
            // }

            // grav_sig_before = grav_signal;
            // pos_sig_before = pos_signal;
        }

        for (int i = 0; i < ec_slavecount; i++)
        {
            q_[JointMap2[START_N + i]] = q_elmo_[START_N + i];
            q_dot_[JointMap2[START_N + i]] = q_dot_elmo_[START_N + i];
            torque_[JointMap2[START_N + i]] = torque_elmo_[START_N + i];
            q_ext_[JointMap2[START_N + i]] = q_ext_elmo_[START_N + i];
            // joint_state_[JointMap2[START_N + i]] = joint_state_elmo_[START_N + i];

            ElmoMode[START_N + i] = EM_DEFAULT;
        }
        if (g_init_args.ecat_device == 1) // mask5
        {
            shm_msgs_->e1_m[mask++]++;
        }
        else if (g_init_args.ecat_device == 2)
        {
            shm_msgs_->e2_m[mask++]++;
        }
        sendJointStatus();
        if (g_init_args.ecat_device != 0)
            getJointCommand();
        if (g_init_args.ecat_device == 1) // mask6
        {
            shm_msgs_->e1_m[mask++]++;
        }
        else if (g_init_args.ecat_device == 2)
        {
            shm_msgs_->e2_m[mask++]++;
        }
        // printf("%f %f %f %f %f %f\n",q_elmo_[START_N],q_elmo_[START_N+1],q_elmo_[START_N+2],q_elmo_[START_N+3],q_elmo_[START_N+4],q_elmo_[START_N+5]);
        // memcpy(q_desired_elmo_, &shm_msgs_->positionCommand, sizeof(float) * MODEL_DOF);

        // ECAT JOINT COMMAND
        if (g_init_args.ecat_device == 1)
        {
            if (shm_msgs_->safety_reset_upper_signal)
            {
                memset(ElmoSafteyMode, 0, sizeof(int) * ELMO_DOF);
                fprintf(stdout, "ELMO 1 : SAFETY RESET \n");
                shm_msgs_->safety_reset_upper_signal = false;
            }
        }

        if (g_init_args.ecat_device == 2)
        {
            if (shm_msgs_->safety_reset_lower_signal)
            {
                memset(ElmoSafteyMode, 0, sizeof(int) * ELMO_DOF);
                fprintf(stdout, "ELMO 2 : SAFETY RESET \n");

                shm_msgs_->safety_reset_lower_signal = false;
            }
        }

        // Joint safety checking ..
        //  static int safe_count = 10;

        // if (safe_count-- < 0)
        // {
        if (!shm_msgs_->safety_disable)
            checkJointSafety();
        // }

        // ECAT JOINT COMMAND
        for (int i = 0; i < ec_slavecount; i++)
        {

            if (g_init_args.ecat_device == 0)
            {
                if (pos_hold_switch)
                {
                    torque_desired_elmo_[START_N + i] = (q_desired_elmo_[START_N + i] - q_elmo_[START_N + i]) * pos_p_gain[JointMap2[START_N + i]] - q_dot_elmo_[START_N + i] * pos_d_gain[JointMap2[START_N + i]];
                    maxTorque = 1000;
                    ElmoMode[START_N + i] = EM_TORQUE;
                }
            }
            // txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
            // txPDO[i]->targetTorque = (int)(torque_desired_elmo_[START_N + i] * NM2CNT[START_N + i] * elmo_axis_direction[START_N + i]);
            // txPDO[i]->maxTorque = (uint16)maxTorque;

            if (ElmoMode[START_N + i] == EM_POSITION)
            {
                txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousPositionmode;
                txPDO[i]->targetPosition = (int)(elmo_axis_direction[START_N + i] * RAD2CNT[START_N + i] * (q_desired_elmo_[START_N + i] + q_zero_elmo_[START_N + i]));
                txPDO[i]->maxTorque = (uint16)maxTorque; // originaly 1000
            }
            else if (ElmoMode[START_N + i] == EM_TORQUE)
            {

                txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
                txPDO[i]->targetTorque = (int)(torque_desired_elmo_[START_N + i] * NM2CNT[START_N + i] * elmo_axis_direction[START_N + i]);

                txPDO[i]->maxTorque = (uint16)maxTorque; // originaly 1000
                if (g_init_args.ecat_device == 1)
                {
                    txPDO[i]->maxTorque = (uint16)(maxTorque * 0.6); // originaly 1000
                }
            }
            else
            {
                txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
                txPDO[i]->targetTorque = (int)0;
            }
        }

        if (de_emergency_off)
            emergencyOff();

#ifdef ECAT_DC
        cur_dc32 = (uint32_t)(ec_DCtime & 0xffffffff);
        if (cur_dc32 > pre_dc32)
            diff_dc32 = cur_dc32 - pre_dc32;
        else
            diff_dc32 = (0xffffffff - pre_dc32) + cur_dc32;
        pre_dc32 = cur_dc32;

        cur_DCtime += diff_dc32;

        ec_sync(cur_DCtime, period_ns, toff);

        if (cur_DCtime > max_DCtime)
            max_DCtime = cur_DCtime;

        shm_msgs_->low_toff = toff;
#endif
        for (int i = 0; i < ec_slavecount; i++)
        {
            shm_msgs_->elmo_torque[JointMap2[START_N + i]] = txPDO[i]->targetTorque;
        }
        ts.tv_nsec += PRNS + toff;
        if (ts.tv_nsec >= SEC_IN_NSEC)
        {
            ts.tv_sec++;
            ts.tv_nsec -= SEC_IN_NSEC;
        }

        total1 += lat_ns;

        total2 += sat_ns;

        total3 += rat_ns;

        if (lat_ns > PERIOD_OVF * 1000)
        {
            int lat_temp = lat_ns;

            while (lat_temp > 500 * 1000)
            {
                cycle_count++;
                ts.tv_nsec += PRNS + toff;
                if (ts.tv_nsec >= SEC_IN_NSEC)
                {
                    ts.tv_sec++;
                    ts.tv_nsec -= SEC_IN_NSEC;
                }

                lat_temp = lat_temp - 500 * 1000;
            }
            // int cycle_mod = lat_ns
            // lat_ns/500000

            l_ovf++;
            printf("ECAT %d : lat %d ns ovf at %ld\n", g_init_args.ecat_device, lat_ns, cycle_count);
        }
        if (sat_ns > 350 * 1000)
        {
            s_ovf++;
            printf("ECAT %d : sat %d ns rsat %d ns ,ovf at %ld\n", g_init_args.ecat_device, sat_ns, rsat_ns, cycle_count);
        }

        static int c_count = 0;
        c_count++;

        lat_avg = total1 / c_count;
        send_avg = total2 / c_count;
        rat_avg = total3 / c_count;

        if (lmax < lat_ns)
        {
            lmax = lat_ns;
        }
        if (smax < sat_ns)
        {
            smax = sat_ns;
        }
        if (lamax < lat_ns)
        {
            lamax = lat_ns;
        }
        if (samax < sat_ns)
        {
            samax = sat_ns;
        }
        if (rmax < rat_ns)
        {
            rmax = rat_ns;
        }

        total_dev2 += sqrt(((sat - savg) * (sat - savg)));
        sdev = total_dev2 / cycle_count;

        // shm_msgs_->send_avg2 = savg;
        // shm_msgs_->send_max2 = smax;
        // shm_msgs_->send_min2 = smin;
        // shm_msgs_->send_dev2 = sdev;

        r_us = rat_ns / 1000;
        if (r_us < 0)
            r_us = 0;
        s_us = sat_ns / 1000;
        if (s_us < 0)
            s_us = 0;
        l_us = lat_ns / 1000;
        if (l_us < 0)
            l_us = 0;

        if (g_init_args.ecat_device == 0)
        {
            if (cycle_count % 2000 == 0)
            {
                // printf("%d : %ld L avg : %5.2f max : %5.2f amax : %5.2f C avg : %5.2f max : %5.2f amax : %5.2f ovf : %d\n ", g_init_args.ecat_device, cycle_count / 2000, lat_avg / 1000.0, lmax / 1000.0, lamax / 1000.0, send_avg / 1000.0, smax / 1000.0, samax / 1000.0, s_ovf);
                // printf("  rcv max : %7.3f ovf : %d mid max : %7.3f ovf : %d snd max : %7.3f ovf : %d statwrd chg cnt : %d\n", low_rcv_max / 1000.0, low_rcv_ovf, low_mid_max / 1000.0, low_mid_ovf, low_snd_max / 1000.0, low_snd_ovf, status_changed_count);

                shm_msgs_->lat_avg = lat_avg;
                shm_msgs_->lat_max = lmax;
                shm_msgs_->lat_max2 = lamax;
                shm_msgs_->lat_ovf = l_ovf;

                shm_msgs_->send_avg = send_avg;
                shm_msgs_->send_max = smax;
                shm_msgs_->send_max2 = samax;
                shm_msgs_->send_ovf = s_ovf;

                c_count = 0;
                total1 = 0;
                total2 = 0;
                lmax = 0;
                smax = 0;
            }
        }
        else if (g_init_args.ecat_device == 1)
        {
            shm_msgs_->lat_avg = lat_avg;
            shm_msgs_->lat_max = lamax;
            shm_msgs_->lat_ovf = l_ovf;
            shm_msgs_->send_avg = send_avg;
            shm_msgs_->send_max = samax;
            shm_msgs_->send_ovf = s_ovf;
            shm_msgs_->rcv_avg = rat_avg;
            shm_msgs_->rcv_max = rmax;

            int max_us = 29;

            shm_msgs_->r_us = r_us;
            shm_msgs_->l_us = l_us;
            shm_msgs_->s_us = s_us;

            if (r_us >= 0 && r_us < max_us)
            {
                shm_msgs_->send_h[r_us]++;
            }
            else if (r_us >= max_us)
            {
                shm_msgs_->send_h[max_us]++;
            }

            if (l_us >= 0 && l_us < max_us)
            {
                shm_msgs_->lat_h[l_us]++;
            }
            else if (l_us >= max_us)
            {
                shm_msgs_->lat_h[max_us]++;
            }

            if (s_us >= 0 && s_us < max_us)
            {
                shm_msgs_->rcv_h[s_us]++;
            }
            else if (s_us >= max_us)
            {
                shm_msgs_->rcv_h[max_us]++;
            }

            // if (r_us < 250)
            // {
            //     if (r_histogram.size() < r_us)
            //     {
            //         r_histogram.resize(r_us + 1, 0);
            //     }
            //     r_histogram[r_us]++;
            // }
            // else
            // {
            //     printf("E2 r ovf with %d us", r_us);
            //     r_ovf_list.push_back(r_us);
            //     rh_ovf++;
            // }

            // if (l_us < 250)
            // {
            //     if (l_histogram.size() < l_us)
            //     {
            //         l_histogram.resize(l_us + 1, 0);
            //     }
            //     l_histogram[l_us]++;
            // }
            // else
            // {
            //     printf("E2 l ovf with %d us", l_us);
            //     l_ovf_list.push_back(l_us);
            //     lh_ovf++;
            // }

            // if (s_us < 250)
            // {
            //     if (s_histogram.size() < s_us)
            //     {
            //         s_histogram.resize(s_us + 1, 0);
            //     }
            //     s_histogram[s_us]++;
            // }
            // else
            // {
            //     printf("E2 s ovf with %d us", s_us);
            //     s_ovf_list.push_back(s_us);
            //     sh_ovf++;
            // }
        }
        else if (g_init_args.ecat_device == 2)
        {
            shm_msgs_->lat_avg2 = lat_avg;
            shm_msgs_->lat_max2 = lamax;
            shm_msgs_->lat_ovf2 = l_ovf;
            shm_msgs_->send_avg2 = send_avg;
            shm_msgs_->send_max2 = samax;
            shm_msgs_->send_ovf2 = s_ovf;
            shm_msgs_->rcv_avg2 = rat_avg;
            shm_msgs_->rcv_max2 = rmax;
            int max_us = 29;

            shm_msgs_->r_us2 = r_us;
            shm_msgs_->l_us2 = l_us;
            shm_msgs_->s_us2 = s_us;

            if (r_us >= 0 && r_us < max_us)
            {
                shm_msgs_->send2_h[r_us]++;
            }
            else if (r_us >= max_us)
            {
                shm_msgs_->send2_h[max_us]++;
            }

            if (l_us >= 0 && l_us < max_us)
            {
                shm_msgs_->lat2_h[l_us]++;
            }
            else if (l_us >= max_us)
            {
                shm_msgs_->lat2_h[max_us]++;
            }

            if (s_us >= 0 && s_us < max_us)
            {
                shm_msgs_->rcv2_h[s_us]++;
            }
            else if (s_us >= max_us)
            {
                shm_msgs_->rcv2_h[max_us]++;
            }

            // if (r_us < 250)
            // {
            //     if (r_histogram.size() < r_us)
            //     {
            //         r_histogram.resize(r_us + 1, 0);
            //     }
            //     r_histogram[r_us]++;
            // }
            // else
            // {
            //     printf("E2 r ovf with %d us", r_us);
            //     r_ovf_list.push_back(r_us);
            //     rh_ovf++;
            // }

            // if (l_us < 250)
            // {
            //     if (l_histogram.size() < l_us)
            //     {
            //         l_histogram.resize(l_us + 1, 0);
            //     }
            //     l_histogram[l_us]++;
            // }
            // else
            // {
            //     printf("E2 l ovf with %d us", l_us);
            //     l_ovf_list.push_back(l_us);

            //     lh_ovf++;
            // }

            // if (s_us < 250)
            // {
            //     if (s_histogram.size() < s_us)
            //     {
            //         s_histogram.resize(s_us + 1, 0);
            //     }
            //     s_histogram[s_us]++;
            // }
            // else
            // {
            //     printf("E2 s ovf with %d us", s_us);
            //     s_ovf_list.push_back(s_us);

            //     sh_ovf++;
            // }
        }

        cycle_count++;
    }

    inOP = FALSE;

    printf("\nELMO : Request init state for all slaves\n");
    /** request INIT state for all slaves
     *  slave number = 0 -> write to all slaves
     */
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);

    printf("ELMO : Checking EC STATE ... \n");
    ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
    printf("ELMO : Checking EC STATE Complete \n");

    // usleep(1000000 * g_init_args.ecat_device);

    // printf("================================================================\n");

    // printf("ELMO %d DIAGNOSE : \n", g_init_args.ecat_device);
    // printf("RECEIVE TOTAL OVF : %d\n", rh_ovf);

    // int h_length = r_histogram.size();
    // int length = 0;
    // for (int i = 0; i < h_length; i++)
    // {
    //     if (r_histogram[i] != 0)
    //     {

    //         length = intlength(r_histogram[i]) - intlength(i);

    //         if (length > 0)
    //         {
    //             for (int j = 0; j < length; j++)
    //             {
    //                 printf(" ");
    //             }
    //         }

    //         printf("%d ", i);
    //     }
    // }
    // printf("\n");

    // for (int i = 0; i < h_length; i++)
    // {
    //     if (r_histogram[i] != 0)
    //     {
    //         length = intlength(r_histogram[i]) - intlength(i);

    //         if (length < 0)
    //         {
    //             for (int i = 0; i < -length; i++)
    //             {
    //                 printf(" ");
    //             }
    //         }
    //         printf("%d ", r_histogram[i]);
    //     }
    // }
    // printf("\n");

    // if(rh_ovf > 0)
    // {
    //     printf("OVF LIST : ");

    //     for(int i=0;i<r_ovf_list.size();i++)
    //     {
    //         printf("  %d",r_ovf_list[i]);
    //     }
    // }

    // printf("\n");

    // printf("SEND TOTAL OVF : %d\n", sh_ovf);
    // h_length = s_histogram.size();
    // length = 0;
    // for (int i = 0; i < h_length; i++)
    // {

    //     if (s_histogram[i] != 0)
    //     {
    //         length = intlength(s_histogram[i]) - intlength(i);

    //         if (length > 0)
    //         {
    //             for (int j = 0; j < length; j++)
    //             {
    //                 printf(" ");
    //             }
    //         }

    //         printf("%d ", i);
    //     }
    // }

    // printf("\n");
    // for (int i = 0; i < h_length; i++)
    // {
    //     if (s_histogram[i] != 0)
    //     {
    //         length = intlength(s_histogram[i]) - intlength(i);
    //         if (length < 0)
    //         {
    //             for (int j = 0; j < -length; j++)
    //             {
    //                 printf(" ");
    //             }
    //         }
    //         printf("%d ", s_histogram[i]);
    //     }
    // }
    // printf("\n");

    // if(sh_ovf > 0)
    // {
    //     printf("OVF LIST : ");

    //     for(int i=0;i<s_ovf_list.size();i++)
    //     {
    //         printf("  %d",s_ovf_list[i]);
    //     }
    // }

    // printf("\n");

    // printf("LATENCY TOTAL OVF : %d\n", lh_ovf);
    // h_length = l_histogram.size();
    // length = 0;
    // for (int i = 0; i < h_length; i++)
    // {
    //     if (l_histogram[i] != 0)
    //     {

    //         length = intlength(l_histogram[i]) - intlength(i);

    //         if (length > 0)
    //         {
    //             for (int j = 0; j < length; j++)
    //             {
    //                 printf(" ");
    //             }
    //         }

    //         printf("%d ", i);
    //     }
    // }

    // printf("\n");
    // for (int i = 0; i < h_length; i++)
    // {
    //     if (l_histogram[i] != 0)
    //     {
    //         length = intlength(l_histogram[i]) - intlength(i);
    //         if (length < 0)
    //         {
    //             for (int j = 0; j < -length; j++)
    //             {
    //                 printf(" ");
    //             }
    //         }
    //         printf("%d ", l_histogram[i]);
    //     }
    // }
    // printf("\n");
    // if(lh_ovf > 0)
    // {
    //     printf("OVF LIST : ");

    //     for(int i=0;i<l_ovf_list.size();i++)
    //     {
    //         printf("  %d",l_ovf_list[i]);
    //     }
    // }

    // printf("\n");
    // printf("================================================================\n");

    return (void *)NULL;
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

bool controlWordGenerate(const uint16_t statusWord, uint16_t &controlWord)
{
    if (!(statusWord & (1 << OPERATION_ENABLE_BIT)))
    {
        if (!(statusWord & (1 << SWITCHED_ON_BIT)))
        {
            if (!(statusWord & (1 << READY_TO_SWITCH_ON_BIT)))
            {
                if (statusWord & (1 << FAULT_BIT))
                {
                    controlWord = 0x80;
                    return false;
                }
                else
                {
                    controlWord = CW_SHUTDOWN;
                    return false;
                }
            }
            else
            {
                controlWord = CW_SWITCHON;
                return true;
            }
        }
        else
        {
            controlWord = CW_ENABLEOP;
            return true;
        }
    }
    else
    {
        controlWord = CW_ENABLEOP;
        return true;
    }
    controlWord = 0;
    return false;
}

void checkJointSafety()
{
    for (int i = 0; i < g_init_args.ecat_slave_num; i++)
    {

        if (ElmoSafteyMode[START_N + i] == 0)
        {
            state_safety_[JointMap2[START_N + i]] = SSTATE::SAFETY_OK;

            if ((joint_lower_limit[START_N + i] > q_elmo_[START_N + i]))
            {
                fprintf(stdout, "%sELMO %d %s : SAFETY LOCK - JOINT LIMIT : %f LIMIT : %f %s\n", cred, g_init_args.ecat_device, ELMO_NAME[START_N + i], q_elmo_[START_N + i], joint_lower_limit[START_N + i], creset);
                state_safety_[JointMap2[START_N + i]] = SSTATE::SAFETY_JOINT_LIMIT;
                ElmoSafteyMode[START_N + i] = 1;
            }
            else if ((joint_upper_limit[START_N + i] < q_elmo_[START_N + i]))
            {
                fprintf(stdout, "%sELMO %d %s : SAFETY LOCK - JOINT LIMIT : %f LIMIT : %f %s\n", cred, g_init_args.ecat_device, ELMO_NAME[START_N + i], q_elmo_[START_N + i], joint_upper_limit[START_N + i], creset);

                state_safety_[JointMap2[START_N + i]] = SSTATE::SAFETY_JOINT_LIMIT;
                ElmoSafteyMode[START_N + i] = 1;
            }

            if (joint_velocity_limit[START_N + i] < abs(q_dot_elmo_[START_N + i]))
            {
                fprintf(stdout, "%sELMO %d %s : SAFETY LOCK - VELOCITY LIMIT %s\n", cred, g_init_args.ecat_device, ELMO_NAME[START_N + i], creset);
                state_safety_[JointMap2[START_N + i]] = SSTATE::SAFETY_VELOCITY_LIMIT;
                ElmoSafteyMode[START_N + i] = 1;
            }
        }

        if (ElmoSafteyMode[START_N + i] == 1)
        {
            q_desired_elmo_[START_N + i] = q_elmo_[START_N + i];
            ElmoMode[START_N + i] = EM_POSITION;
            ElmoSafteyMode[START_N + i] = 2;
        }

        if (ElmoSafteyMode[START_N + i] == 2)
        {
            ElmoMode[START_N + i] = EM_POSITION;
        }
    }

    fflush(stdout);
}

void checkJointStatus()
{
}

// void initSharedMemory()
// {

//     if ((shm_id_ = shmget(shm_msg_key, sizeof(SHMmsgs), IPC_CREAT | 0666)) == -1)
//     {
//         std::printf("shm mtx failed \n");
//         exit(0);
//     }

//     if ((shm_msgs_ = (SHMmsgs *)shmat(shm_id_, NULL, 0)) == (SHMmsgs *)-1)
//     {
//         std::printf("shmat failed \n");
//         exit(0);
//     }
//     /*
//     if (pthread_mutexattr_init(&shm_msgs_->mutexAttr) == 0)
//     {
//         std::printf("shared mutex attr init\n");
//     }

//     if (pthread_mutexattr_setpshared(&shm_msgs_->mutexAttr, PTHREAD_PROCESS_SHARED) == 0)
//     {
//         std::printf("shared mutexattr set\n");
//     }

//     if (pthread_mutex_init(&shm_msgs_->mutex, &shm_msgs_->mutexAttr) == 0)
//     {
//         std::printf("shared mutex init\n");
//     }*/

//     if (shmctl(shm_id_, SHM_LOCK, NULL) == 0)
//     {
//         //std::printf("SHM_LOCK enabled\n");
//     }
//     else
//     {
//         std::printf("SHM lock failed\n");
//     }

//     shm_msgs_->t_cnt = 0;
//     shm_msgs_->controllerReady = false;
//     shm_msgs_->statusWriting = 0;
//     shm_msgs_->commanding = false;
//     shm_msgs_->reading = false;
//     shm_msgs_->shutdown = false;

//     //
//     //float lat_avg, lat_min, lat_max, lat_dev;
//     //float send_avg, send_min, send_max, send_dev;

//     shm_msgs_->lat_avg = 0;
//     shm_msgs_->lat_min = 0;
//     shm_msgs_->lat_max = 100000;
//     shm_msgs_->lat_dev = 0;

//     shm_msgs_->send_avg = 0;
//     shm_msgs_->send_min = 0;
//     shm_msgs_->send_max = 100000;
//     shm_msgs_->send_dev = 0;
// }
void sendJointStatus()
{

    shm_msgs_->statusWriting++;

    memcpy(&shm_msgs_->pos[Q_START], &q_[Q_START], sizeof(float) * PART_ELMO_DOF);
    memcpy(&shm_msgs_->posExt[Q_START], &q_ext_[Q_START], sizeof(float) * PART_ELMO_DOF);
    memcpy(&shm_msgs_->vel[Q_START], &q_dot_[Q_START], sizeof(float) * PART_ELMO_DOF);
    memcpy(&shm_msgs_->torqueActual[Q_START], &torque_[Q_START], sizeof(float) * PART_ELMO_DOF);

    memcpy(&shm_msgs_->safety_status[Q_START], &state_safety_[Q_START], sizeof(int8_t) * PART_ELMO_DOF);
    memcpy(&shm_msgs_->zp_status[Q_START], &state_zp_[Q_START], sizeof(int8_t) * PART_ELMO_DOF);
    memcpy(&shm_msgs_->ecat_status[Q_START], &state_elmo_[Q_START], sizeof(int8_t) * PART_ELMO_DOF);

    shm_msgs_->statusWriting--;

    if (g_init_args.ecat_device == 1)
    {
        shm_msgs_->statusCount = cycle_count; //,memory_order_release);
    }
    // // TODO: 2
    if (g_init_args.ecat_device == 2)
        shm_msgs_->statusCount2 = cycle_count; //,memory_order_release);

    // shm_msgs_->statusCount = cycle_count;

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
    // while (shm_msgs_->commanding.load(std::memory_order_acquire))
    // {
    //     clock_nanosleep(CLOCK_MONOTONIC, 0, &ts_us1, NULL);
    // }

    cpu_relax();

    static int stloop;
    static bool stloop_check;
    stloop_check = false;
    if (stloop == shm_msgs_->stloopCount)
    {
        stloop_check = true;
    }
    stloop = shm_msgs_->stloopCount;
    static int commandCount;
    int wait_tick;

    // if (!stloop_check)
    // {
    //     while (shm_msgs_->commandCount == commandCount)
    //     {
    //         clock_nanosleep(CLOCK_MONOTONIC, 0, &ts_us1, NULL);
    //         if (++wait_tick > 3)
    //         {
    //             break;
    //         }
    //     }
    // }

    // memcpy(&command_mode_[Q_START], &shm_msgs_->commandMode[Q_START], sizeof(int) * PART_ELMO_DOF);
    // memcpy(&q_desired_[Q_START], &shm_msgs_->positionCommand[Q_START], sizeof(float) * PART_ELMO_DOF);

    if (g_init_args.ecat_device == 1)
    {
        while (shm_msgs_->cmd_upper)
        {
            clock_nanosleep(CLOCK_MONOTONIC, 0, &ts_us1, NULL);
        };

        if (!shm_msgs_->cmd_upper)
        {
            shm_msgs_->cmd_upper = true;
            memcpy(&torque_desired_[Q_START], &shm_msgs_->torqueCommand[Q_START], sizeof(float) * PART_ELMO_DOF);
            // memcpy(&max_cnt_[Q_START], &shm_msgs_->max_cnt_[Q_START], sizeof(int) * PART_ELMO_DOF);
            shm_msgs_->cmd_upper = false;
        }
    }

    if (g_init_args.ecat_device == 2)
    {
        while (shm_msgs_->cmd_lower)
        {
            clock_nanosleep(CLOCK_MONOTONIC, 0, &ts_us1, NULL);
        };
        shm_msgs_->cmd_lower = true;
        memcpy(&torque_desired_[Q_START], &shm_msgs_->torqueCommand[Q_START], sizeof(float) * PART_ELMO_DOF);
        // memcpy(&max_cnt_[Q_START], &shm_msgs_->max_cnt_[Q_START], sizeof(int) * PART_ELMO_DOF);
        shm_msgs_->cmd_lower = false;
    }

    commandCount = shm_msgs_->commandCount;

    for (int i = 0; i < ec_slavecount; i++)
    {
        // command_mode_elmo_[JointMap[Q_START + i]] = command_mode_[Q_START + i];

        // if (command_mode_[Q_START + i] == 1)
        // {
        torque_desired_elmo_[JointMap[Q_START + i]] = torque_desired_[Q_START + i];
        // max_cnt_elmo_[JointMap[Q_START + i]] = max_cnt_[Q_START + i];

        ElmoMode[START_N + i] = EM_TORQUE;
        // }
        // else if (command_mode_[Q_START + i] == 2)
        // {
        //     q_desired_elmo_[JointMap[Q_START + i]] = q_desired_[Q_START + i];
        // }
    }

    static int commandCount_before = -1;
    static int commandCount_before2 = -1;
    static int errorCount = -2;
    static int errorTimes = 0;

    static bool start_observe = false;

    if (!start_observe)
    {
        if (shm_msgs_->controlModeUpper && shm_msgs_->controlModeUpper)
        {
            start_observe = true;
        }
    }

    if (start_observe)
    {
        if ((shm_msgs_->controlModeUpper && g_init_args.ecat_device == 1) ||
            (shm_msgs_->controlModeLower && g_init_args.ecat_device == 2))
        {

            if (errorTimes == 0)
            {
                if (commandCount <= commandCount_before) // shit
                {
                    if (stloop_check)
                    {
                        errorTimes++;
                        if (errorTimes > 2)
                            fprintf(stdout, "%ld ELMO %d : commandCount Error wit sterror current : %d before : %d \n", (long)control_time_real_ / 1000, g_init_args.ecat_device, commandCount, commandCount_before);
                        // std::cout << control_time_us_ << "ELMO_LOW : commandCount Error current : " << commandCount << " before : " << commandCount_before << std::'\n';
                        // std::cout << "stloop same cnt" << std::'\n';
                    }
                    else
                    {
                        // printf("%ld ELMO %d : commandCount Error without st current : %d before : %d \n", (long)control_time_real_ / 1000, g_init_args.ecat_device, commandCount, commandCount_before);
                    }
                }
            }
            else if (errorTimes > 0)
            {
                if (commandCount > commandCount_before) // no problem
                {
                    errorTimes = 0;
                    errorCount = 0;
                }
                else // shit
                {
                    errorTimes++;

                    if (errorTimes > CL_LOCK)
                    {
                        if (errorCount != commandCount)
                        {
                            fprintf(stdout, "%s %f ELMO %d : commandCount Warn! SAFETY LOCK%s\n", cred, control_time_real_, g_init_args.ecat_device, creset);

                            // std::cout << cred << control_time_us_ << "ELMO_LOW : commandCount Warn! SAFETY LOCK" << creset << std::'\n';

                            // std::fill(ElmoSafteyMode, ElmoSafteyMode + MODEL_DOF, 1);

                            for (int i = 0; i < ec_slavecount; i++)
                            {
                                ElmoSafteyMode[START_N + i] = 1;
                                state_safety_[JointMap2[START_N + i]] = SSTATE::SAFETY_COMMAND_LOCK;
                            }
                            errorCount = commandCount;
                        }
                        else
                        {
                            // std::cout << errorTimes << "ELMO_LOW : commandCount error duplicated" << std::'\n';
                        }
                    }
                }
            }
        }
    }
    else
    {
        shm_msgs_->maxTorque = 0;
    }

    commandCount_before2 = commandCount_before;
    commandCount_before = commandCount;

    maxTorque = shm_msgs_->maxTorque; // Max Torque is defined at State_manager.cpp maxTorque value.
}

bool saveCommutationLog()
{
    FILE *fp;
    fp = fopen(g_init_args.commutation_cache_file, "wb");
    if (fp == NULL)
    {
        printf("ELMO %d : Failed to Open Commutation Log File\n", g_init_args.ecat_device);
        return false;
    }
    else
    {
        printf("ELMO %d : COMMUTATION SAVED!\n", g_init_args.ecat_device);
    }

    struct timespec cm_time;
    clock_gettime(CLOCK_REALTIME, &cm_time);

    fwrite(&cm_time, sizeof(timespec), 1, fp);

    fclose(fp);

    return true;
}

bool loadCommutationLog(struct timespec &commutation_time)
{
    FILE *fp;
    fp = fopen(g_init_args.commutation_cache_file, "rb");
    if (fp == NULL)
    {
        printf("ELMO %d : Failed to Open Commutation Log file\n", g_init_args.ecat_device);
        return false;
    }

    struct timespec ts_now;
    clock_gettime(CLOCK_REALTIME, &ts_now);

    int fr = fread(&commutation_time, sizeof(timespec), 1, fp);

    fclose(fp);

    float t_before = (float)(getTimeDiff(commutation_time, ts_now) / SEC_IN_NSEC);

    // printf("ELMO %d : Commutation done %d seconds before \n", g_init_args.ecat_device, (int)t_before);

    return true;
}

bool saveZeroPoint()
{
    FILE *fp;
    fp = fopen(g_init_args.zeropoint_cache_file, "wb");
    if (fp == NULL)
    {
        printf("ELMO %d : Failed to Open zeropoint Log file\n", g_init_args.ecat_device);
        return false;
    }
    struct timespec t_now;
    clock_gettime(CLOCK_REALTIME, &t_now);

    fwrite(&t_now, sizeof(timespec), 1, fp);
    fwrite(q_zero_elmo_, sizeof(double), ELMO_DOF, fp);

    fclose(fp);
    return true;
}

bool loadZeroPoint(bool force)
{
    FILE *fp;
    fp = fopen(g_init_args.zeropoint_cache_file, "rb");
    if (fp == NULL)
    {
        printf("ELMO %d : Failed to Open zeropoint Log file\n", g_init_args.ecat_device);
        return false;
    }

    struct timespec ts_now_;
    clock_gettime(CLOCK_REALTIME, &ts_now_);

    struct timespec ts_zp_saved_;
    int fr = fread(&ts_zp_saved_, sizeof(timespec), 1, fp);

    double getzp[ELMO_DOF];
    fr = fread(getzp, sizeof(double), ELMO_DOF, fp);

    fclose(fp);

    if (!force)
    {

        struct timespec commutation_save_time_;

        loadCommutationLog(commutation_save_time_);

        if (getTimeDiff(ts_zp_saved_, commutation_save_time_) > 0)
        {
            printf("ELMO %d : commutation saved time is before than zp saved time\n", g_init_args.ecat_device);
            return false;
        }
    }

    for (int i = 0; i < g_init_args.ecat_slave_num; i++)
    {
        state_zp_[JointMap2[START_N + i]] = ZSTATE::ZP_SUCCESS;
        q_zero_elmo_[START_N + i] = getzp[START_N + i];
    }

    return true;
}

void emergencyOff() // TorqueZero
{
    for (int i = 0; i < ec_slavecount; i++)
    {
        txPDO[i]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousTorquemode;
        txPDO[i]->targetTorque = (int)0;
    }
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

int getElmoState(uint16_t state_bit)
{
    if (!(state_bit & (1 << OPERATION_ENABLE_BIT)))
    {
        if (!(state_bit & (1 << SWITCHED_ON_BIT)))
        {
            if (!(state_bit & (1 << READY_TO_SWITCH_ON_BIT)))
            {
                if (state_bit & (1 << FAULT_BIT))
                {
                    return ELMO_FAULT;
                }
                else
                {
                    return ELMO_NOTFAULT;
                }
            }
            else
            {
                return ELMO_READY_TO_SWITCH_ON;
            }
        }
        else
        {
            return ELMO_SWITCHED_ON;
        }
    }
    else
    {
        return ELMO_OPERATION_ENABLE;
    }
}

void findzeroLeg()
{
    for (int i = 0; i < 6; i++)
    {
        q_zero_elmo_[i + R_HipYaw_Joint] = q_elmo_[i + R_HipYaw_Joint] - q_ext_elmo_[i + R_HipYaw_Joint];
        // pub_to_gui(dc, "jointzp %d %d", i + R_HipYaw_Joint, 1);
        q_zero_elmo_[i + L_HipYaw_Joint] = q_elmo_[i + L_HipYaw_Joint] - q_ext_elmo_[i + L_HipYaw_Joint];
        // pub_to_gui(dc, "jointzp %d %d", i + TOCABI::L_HipYaw_Joint, 1);
        // std::cout << ELMO_NAME[i + R_HipRoll_Joint] << " pz IE P : " << q_elmo_[i + R_HipRoll_Joint] << " pz EE P : " << q_ext_elmo_[i + R_HipRoll_Joint] << '\n';
        // std::cout << ELMO_NAME[i + L_HipRoll_Joint] << " pz ELMO : " << q_elmo_[i + L_HipRoll_Joint] << " pz ELMO : " << q_ext_elmo_[i + L_HipRoll_Joint] << '\n';
    }
}

void findZeroPointlow(int slv_number, double time_real_)
{
    double velocity = 0.1;
    double fztime = 1.0;
    if (elmofz[slv_number].findZeroSequence == FZ_CHECKHOMMINGSTATUS)
    {

        state_zp_[JointMap2[slv_number]] = ZSTATE::ZP_SEARCHING_ZP;
        elmofz[slv_number].initTime = time_real_;
        elmofz[slv_number].initPos = q_elmo_[slv_number];
        elmofz[slv_number].desPos = q_ext_elmo_[slv_number];
        elmofz[slv_number].trajTime = abs(q_ext_elmo_[slv_number] / velocity);

        if (elmofz[slv_number].trajTime < 0.5)
            elmofz[slv_number].trajTime = 0.5;

        elmofz[slv_number].findZeroSequence = FZ_FINDHOMMINGSTART;

        if (q_ext_elmo_[slv_number] > 0)
        {
            elmofz[slv_number].init_direction = -1;
        }
        else
        {
            elmofz[slv_number].init_direction = 1;
        }

        if ((q_ext_elmo_[slv_number] > 3.14) || (q_ext_elmo_[slv_number] < -3.14))
        {
            printf("%sELMO %d : REBOOTING REQUIRED! joint %d : ext encoder error%s\n", cred, g_init_args.ecat_device, slv_number, creset);
            // std::cout << cred << "elmo reboot required. joint " << slv_number << "external encoder error" << q_ext_elmo_[slv_number] << '\n';
        }
    }

    if (elmofz[slv_number].findZeroSequence == FZ_FINDHOMMINGSTART)
    {

        ElmoMode[slv_number] = EM_POSITION;
        q_desired_elmo_[slv_number] = elmoJointMove(time_real_, elmofz[slv_number].initPos, -elmofz[slv_number].desPos, elmofz[slv_number].initTime, elmofz[slv_number].trajTime);

        if (time_real_ == elmofz[slv_number].initTime)
        {
            // printf("joint %d init pos : %f " << slv_number << "  init pos : " << elmofz[slv_number].initPos << "   goto " << elmofz[slv_number].initPos + elmofz[slv_number].init_direction * 0.6 << '\n';
        }
        static int sucnum = 0;

        if ((time_real_ >= (elmofz[slv_number].initTime + elmofz[slv_number].trajTime)) && (time_real_ <= (elmofz[slv_number].initTime + elmofz[slv_number].trajTime + 1.0)))
        {
            if (abs(q_ext_elmo_[slv_number]) < 1.0E-6)
            {
                elmofz[slv_number].findZeroSequence = FZ_FINDHOMMINGEND;
                elmofz[slv_number].result = ElmoHommingStatus::SUCCESS;
                sucnum++;
                state_zp_[JointMap2[slv_number]] = ZSTATE::ZP_SUCCESS;
                printf("joint %d leg success\n", slv_number);
            }
        }
        else if (time_real_ > (elmofz[slv_number].initTime + elmofz[slv_number].trajTime + 1.0))
        {
            elmofz[slv_number].initTime = time_real_;
            elmofz[slv_number].initPos = q_elmo_[slv_number];
            elmofz[slv_number].desPos = q_ext_elmo_[slv_number];
            elmofz[slv_number].findZeroSequence = FZ_FINDHOMMINGSTART;
        }
        // if (control_time_real_ > elmofz[slv_number].initTime + fztime * 4.0)
        // {
        //     elmofz[slv_number].result == ElmoHommingStatus::FAILURE;
        // }
    }
}

void findZeroPoint(int slv_number, double time_now_)
{
    // double fztime = 3.0;
    double fztime_manual = 300.0;
    if (elmofz[slv_number].findZeroSequence == FZ_CHECKHOMMINGSTATUS) // Check initial status of homming sensor
    {
        state_zp_[JointMap2[slv_number]] = ZSTATE::ZP_SEARCHING_ZP;
        // pub_to_gui(dc, "jointzp %d %d", slv_number, 0);
        if (hommingElmo[slv_number]) // Initial status of homming sensor is TRUE
        {
            // printf("init homming on goto findhomming start : %d %7.3f\n", slv_number, time_now_);
            elmofz[slv_number].findZeroSequence = FZ_FINDHOMMINGSTART;
            elmofz[slv_number].initTime = time_now_;
            elmofz[slv_number].initPos = q_elmo_[slv_number];
            elmofz[slv_number].firstPos = q_elmo_[slv_number];
        }
        else // Initial status of homming sensor if FALSE
        {

            // printf("init homming off goto findhomming : %d %7.3f\n", slv_number, time_now_);
            elmofz[slv_number].findZeroSequence = FZ_FINDHOMMING;
            elmofz[slv_number].initTime = time_now_;
            elmofz[slv_number].initPos = q_elmo_[slv_number];
            elmofz[slv_number].firstPos = q_elmo_[slv_number];
        }
    }
    else if (elmofz[slv_number].findZeroSequence == FZ_FINDHOMMINGSTART)
    {
        // go to + 0.3rad until homming sensor turn off
        ElmoMode[slv_number] = EM_POSITION;
        q_desired_elmo_[slv_number] = elmoJointMove(time_now_, elmofz[slv_number].initPos, elmofz[slv_number].hommingLength, elmofz[slv_number].initTime, elmofz[slv_number].fztime);

        if (!elmofz[slv_number].startFound)
        {
            if ((hommingElmo[slv_number] == 0) && (hommingElmo_before[slv_number] == 0)) // IF HOMMING IS OFF
            {

                elmofz[slv_number].pos_turnedoff = q_elmo_[slv_number];
                elmofz[slv_number].startFound = true;
                elmofz[slv_number].posStart = q_elmo_[slv_number];

                // printf("%s findhommingstart : homming is off! continue to search blank : %d q : %f\n", ELMO_NAME[slv_number], slv_number, q_elmo_[slv_number]);

                // printf("goto homming off : %d %7.3f\n", slv_number, time_now_);
                // std::printf("motor " << slv_number << " seq 1 complete, wait 1 sec\n");
                hommingElmo_before[slv_number] = hommingElmo[slv_number];
            }
        }
        else
        {
            if ((hommingElmo[slv_number] == 0) && (hommingElmo_before[slv_number] == 0)) // IF HOMMING IS OFF CONT..
            {
                if (q_elmo_[slv_number] > elmofz[slv_number].pos_turnedoff + elmofz[slv_number].min_black_length)
                {
                    // printf("%s findhommingstart : end searching blank! GOTO FIND HOMMING END : %d q : %f\n", ELMO_NAME[slv_number], slv_number, q_elmo_[slv_number]);

                    elmofz[slv_number].findZeroSequence = FZ_FINDHOMMINGEND;
                    elmofz[slv_number].initTime = time_now_;
                    elmofz[slv_number].startFound = false;
                    elmofz[slv_number].initPos = q_elmo_[slv_number];
                    elmofz[slv_number].pos_turnedoff = q_elmo_[slv_number];
                }

                // printf("goto homming off : %d %7.3f\n", slv_number, time_now_);
                // std::printf("motor " << slv_number << " seq 1 complete, wait 1 sec\n");
                hommingElmo_before[slv_number] = hommingElmo[slv_number];
            }
            else if ((hommingElmo[slv_number] == 1) && (hommingElmo_before[slv_number] == 0)) // HOMMING IS TURNED ON?! ANOTHER SEGMENT!
            {
                printf("%s findhommingstart : homming on is detected while searching blank! : %d q : %f\n", ELMO_NAME[slv_number], slv_number, q_elmo_[slv_number]);

                elmofz[slv_number].pos_turnedon = q_elmo_[slv_number];
                elmofz[slv_number].startFound = false;
            }
        }

        if (time_now_ > elmofz[slv_number].initTime + elmofz[slv_number].fztime)
        {
            printf("%s ELMO %d : WARNING! : %s homming not turning off! GOTO FAILURE %s\n", cred, g_init_args.ecat_device, ELMO_NAME[slv_number], creset);
            state_zp_[JointMap2[slv_number]] = ZSTATE::ZP_NOT_ENOUGH_HOMMING;
            elmofz[slv_number].startFound = false;
            elmofz[slv_number].findZeroSequence = 7;
            elmofz[slv_number].result = ElmoHommingStatus::FAILURE;
            elmofz[slv_number].initTime = time_now_;
        }

        // if joint is head yaw, some logics must be added.
    }
    else if (elmofz[slv_number].findZeroSequence == FZ_FINDHOMMINGEND)
    {
        // printf("%f goto homming on : %d\n", time_now_, slv_number);
        ElmoMode[slv_number] = EM_POSITION;
        q_desired_elmo_[slv_number] = elmoJointMove(time_now_, elmofz[slv_number].initPos, -elmofz[slv_number].findEndLength, elmofz[slv_number].initTime, elmofz[slv_number].fztime * 2.0);

        if (!elmofz[slv_number].endFound)
        {
            if ((hommingElmo_before[slv_number] == 1) && (hommingElmo[slv_number] == 0))
            {
                elmofz[slv_number].posEnd = q_elmo_[slv_number];
                elmofz[slv_number].endFound = 1;
                elmofz[slv_number].pos_turnedoff = q_elmo_[slv_number];

                // printf("%s FZ_FINDHOMMINGEND : Homming is off! END FOUND? CONTINUE to search blank : %d q : %f\n", ELMO_NAME[slv_number], slv_number, q_elmo_[slv_number]);
            }
        }
        else
        {
            // if end found,
            if ((hommingElmo_before[slv_number] == 0) && (hommingElmo[slv_number] == 0))
            {

                if (q_elmo_[slv_number] < elmofz[slv_number].pos_turnedoff - elmofz[slv_number].min_black_length)
                {
                    // it's clean!
                    // check the seq
                    //  printf("goto homming on : %d\n", slv_number);

                    // printf("%s FZ_FINDHOMMINGEND : IT IS CLEAN! CHECK THE DISTANCE : %d q : %f\n", ELMO_NAME[slv_number], slv_number, q_elmo_[slv_number]);

                    if (abs(elmofz[slv_number].posStart - elmofz[slv_number].pos_turnedoff) > elmofz[slv_number].req_length)
                    {
                        // printf("%s FZ_FINDHOMMINGEND : CHECK OK GOTO ZP : %d q : %f\n", ELMO_NAME[slv_number], slv_number, q_elmo_[slv_number]);

                        elmofz[slv_number].posEnd = q_elmo_[slv_number];
                        elmofz[slv_number].findZeroSequence = FZ_GOTOZEROPOINT;
                        elmofz[slv_number].endFound = false;
                        state_zp_[JointMap2[slv_number]] = ZSTATE::ZP_GOTO_ZERO;
                        elmofz[slv_number].initPos = q_elmo_[slv_number];
                        q_zero_elmo_[slv_number] = (elmofz[slv_number].posEnd + elmofz[slv_number].posStart) * 0.5 + q_zero_mod_elmo_[slv_number];
                        elmofz[slv_number].initTime = time_now_;
                    }
                    else
                    {
                        printf("ELMO %d : FZ_FINDHOMMINGEND : Joint %d %s : Not enough distance, required : %f current : %f \n", g_init_args.ecat_device, slv_number, ELMO_NAME[slv_number], elmofz[slv_number].req_length, abs(elmofz[slv_number].posStart - elmofz[slv_number].pos_turnedoff));
                        // std::printf("Joint " << slv_number << " " << ELMO_NAME[slv_number] << " : Not enough distance, required : " << elmofz[slv_number].req_length << ", detected : " << abs(elmofz[slv_number].posStart - q_elmo_[slv_number]) << '\n';

                        // std::printf("Joint " << slv_number << " " << ELMO_NAME[slv_number] << "if you want to proceed with detected length, proceed with manual mode \n");

                        state_zp_[JointMap2[slv_number]] = ZSTATE::ZP_NOT_ENOUGH_HOMMING;
                        elmofz[slv_number].findZeroSequence = 7;
                        elmofz[slv_number].result = ElmoHommingStatus::FAILURE;
                        elmofz[slv_number].initTime = time_now_;
                        elmofz[slv_number].endFound = false;
                    }
                }

                // printf("goto zero : %d\n", slv_number);
            }
            else if ((hommingElmo_before[slv_number] == 0) && (hommingElmo[slv_number] == 1))
            {
                // WHY,?
                printf("%s FZ_FINDHOMMINGEND : THERE IS MORE HOMMING ON WHILE SEARCHng blank : %d q : %f\n", ELMO_NAME[slv_number], slv_number, q_elmo_[slv_number]);

                elmofz[slv_number].endFound = 0;
                elmofz[slv_number].pos_turnedon = q_elmo_[slv_number];
            }
        }
        // go to -20deg until homming turn on, and turn off

        if (time_now_ > elmofz[slv_number].initTime + elmofz[slv_number].fztime * 2.0)
        {
            printf("%s FZ_FINDHOMMINGEND : homming is not turning off TIMEOVER!! goto manual : %d q : %f\n", ELMO_NAME[slv_number], slv_number, q_elmo_[slv_number]);

            // printf("goto seg 6 : %d\n", slv_number);
            // If dection timeout, go to failure sequence
            elmofz[slv_number].initTime = time_now_;
            elmofz[slv_number].findZeroSequence = 7;
            elmofz[slv_number].initPos = q_elmo_[slv_number];
        }
    }
    else if (elmofz[slv_number].findZeroSequence == FZ_FINDHOMMING)
    { // start from unknown

        ElmoMode[slv_number] = EM_POSITION;
        q_desired_elmo_[slv_number] = elmoJointMove(time_now_, elmofz[slv_number].initPos, elmofz[slv_number].init_direction * elmofz[slv_number].hommingLength, elmofz[slv_number].initTime, elmofz[slv_number].fztime);
        if (time_now_ > (elmofz[slv_number].initTime + elmofz[slv_number].fztime))
        {
            // printf("fzhm: %d\n", slv_number);
            q_desired_elmo_[slv_number] = elmoJointMove(time_now_, elmofz[slv_number].initPos + elmofz[slv_number].hommingLength * elmofz[slv_number].init_direction, -0.6 * elmofz[slv_number].init_direction, elmofz[slv_number].initTime + elmofz[slv_number].fztime, elmofz[slv_number].fztime * 2.0);
        }

        if (hommingElmo[slv_number] && hommingElmo_before[slv_number])
        {
            // printf("set to seq 1 : %d\n", slv_number);

            // printf("%s FZ_FINDHOMMING : homming found goto seq 1 : %d q : %f\n", ELMO_NAME[slv_number], slv_number, q_elmo_[slv_number]);

            elmofz[slv_number].findZeroSequence = 1;
            elmofz[slv_number].initTime = time_now_;
            elmofz[slv_number].initPos = q_elmo_[slv_number];
        }

        if (time_now_ > (elmofz[slv_number].initTime + elmofz[slv_number].fztime * 3.0))
        {
            // printf("set to seq 6 : %d\n", slv_number);
            printf("%s FZ_FINDHOMMING : detection timeout! manual! : %d q : %f\n", ELMO_NAME[slv_number], slv_number, q_elmo_[slv_number]);

            elmofz[slv_number].initTime = time_now_;
            elmofz[slv_number].findZeroSequence = 7;
            elmofz[slv_number].initPos = q_elmo_[slv_number];
        }
    }
    else if (elmofz[slv_number].findZeroSequence == FZ_GOTOZEROPOINT)
    {
        ElmoMode[slv_number] = EM_POSITION;

        double go_distance = q_zero_elmo_[slv_number] + q_goinit_[slv_number] - elmofz[slv_number].initPos;
        double go_to_zero_dur = elmofz[slv_number].fztime * (abs(go_distance) / elmofz[slv_number].hommingLength);
        q_desired_elmo_[slv_number] = elmoJointMove(time_now_, elmofz[slv_number].initPos, go_distance, elmofz[slv_number].initTime, go_to_zero_dur);

        // go to zero position
        if (time_now_ > (elmofz[slv_number].initTime + go_to_zero_dur))
        {
            // printf("go to zero complete !\n");
            printf("Motor %d %s : Zero Point Found : %8.6f, homming length : %8.6f ! \n", slv_number, ELMO_NAME[slv_number], q_zero_elmo_[slv_number], abs(elmofz[slv_number].posStart - elmofz[slv_number].posEnd));
            // fflush(stdout);

            // printf("\33[2K\r");
            // for(int i=0;i<16;i++)
            // {

            // }

            // pub_to_gui(dc, "jointzp %d %d", slv_number, 1);
            elmofz[slv_number].result = ElmoHommingStatus::SUCCESS;
            state_zp_[JointMap2[slv_number]] = ZSTATE::ZP_SUCCESS;
            //                                        //std::cout << slv_number << "Start : " << elmofz[slv_number].posStart << "End:" << elmofz[slv_number].posEnd << '\n';
            //                                        //q_desired_elmo_[slv_number] = positionZeroElmo(slv_number);
            elmofz[slv_number].findZeroSequence = 8; // torque to zero -> 8 position hold -> 5
            ElmoMode[slv_number] = EM_TORQUE;
            torque_desired_elmo_[slv_number] = 0.0;
        }
    }
    else if (elmofz[slv_number].findZeroSequence == 5)
    {
        // find zero complete, hold zero position.
        ElmoMode[slv_number] = EM_POSITION;
        q_desired_elmo_[slv_number] = q_zero_elmo_[slv_number];
    }
    else if (elmofz[slv_number].findZeroSequence == 6)
    {
        // find zero point failed
        ElmoMode[slv_number] = EM_POSITION;
        q_desired_elmo_[slv_number] = elmoJointMove(time_now_, elmofz[slv_number].initPos, elmofz[slv_number].firstPos - elmofz[slv_number].initPos, elmofz[slv_number].initTime, elmofz[slv_number].fztime);
        if (time_now_ > (elmofz[slv_number].initTime + elmofz[slv_number].fztime))
        {
            elmofz[slv_number].findZeroSequence = 7;
            printf("Motor %d %s : Zero point detection Failed. Manual Detection Required. \n", slv_number, ELMO_NAME[slv_number]);
            // pub_to_gui(dc, "jointzp %d %d", slv_number, 2);
            elmofz[slv_number].result = ElmoHommingStatus::FAILURE;
            state_zp_[JointMap2[slv_number]] = ZSTATE::ZP_MANUAL_REQUIRED;
            elmofz[slv_number].initTime = time_now_;
        }
    }
    else if (elmofz[slv_number].findZeroSequence == 7)
    {
        ElmoMode[slv_number] = EM_TORQUE;
        torque_desired_elmo_[slv_number] = 0.0;
        if (hommingElmo[slv_number] && hommingElmo_before[slv_number])
        {
            elmofz[slv_number].findZeroSequence = 1;
            elmofz[slv_number].initTime = time_now_;
            elmofz[slv_number].initPos = q_elmo_[slv_number];
        }
        if (time_now_ > (elmofz[slv_number].initTime + fztime_manual))
        {
            printf("Motor %d %s :  Manual Detection Failed. \n", slv_number, ELMO_NAME[slv_number]);
            // pub_to_gui(dc, "jointzp %d %d", slv_number, 3);
            elmofz[slv_number].findZeroSequence = 8;
        }
    }
    else if (elmofz[slv_number].findZeroSequence == 8)
    {
        ElmoMode[slv_number] = EM_TORQUE;
        torque_desired_elmo_[slv_number] = 0.0;
    }
}
