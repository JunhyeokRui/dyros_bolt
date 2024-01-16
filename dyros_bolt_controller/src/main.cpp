#include <ros/ros.h>
#include "dyros_bolt_controller/dyros_bolt_controller.h"


#include <math.h>

#include <signal.h>

#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>

#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <ctime>

volatile bool *prog_shutdown;

void SIGINT_handler(int sig)
{
    std::cout << " CNTRL : shutdown Signal" << std::endl;
    *prog_shutdown = true;
}

int main(int argc, char **argv)
{
    std::cout << std::endl
         << "=====================================" << std::endl;
    std::cout << " CNTRL : Starting DYROS BOLT CONTROLLER! " << std::endl;
    signal(SIGINT, SIGINT_handler);

#ifdef COMPILE_REALROBOT
    mlockall(MCL_CURRENT | MCL_FUTURE);
#endif
    std::cout << "test1" << std::endl;
    ros::init(argc, argv, "dyros_bolt_controller", ros::init_options::NoSigintHandler);
    // ros::NodeHandle nh("~");

    DataContainer dc_;
    
    std::cout << "test2" << std::endl;
    // std::string mode;

    dc_.nh.param("/dyros_bolt_controller/sim_mode", dc_.simMode, false);
    dc_.nh.getParam("/dyros_bolt_controller/Kp", dc_.Kps);
    dc_.nh.getParam("/dyros_bolt_controller/Kv", dc_.Kvs);
    // nh.param<std::string>("run_mode", mode, "simulation");

    std::cout << "test3" << std::endl;
    if (dc_.Kps.size() != MODEL_DOF)
    {
        std::cout << "Kps size error ! " << dc_.Kps.size() << std::endl;
    }
    if (dc_.Kvs.size() != MODEL_DOF)
    {
        std::cout << "Kps size error ! " << dc_.Kvs.size() << std::endl;
    }
    std::cout << "test4" << std::endl;
    // ControlBase *ctr_obj;
    StateManager stm(dc_); //rui -
    std::cout << "test4-1" << std::endl;
    DyrosBoltController tc_(stm); //rui
    std::cout << "test4-2" << std::endl;
    int shm_id_;
    std::cout << "test4-3" << std::endl;
    init_shm(shm_msg_key, shm_id_, &dc_.tc_shm_); //rui - init shared memory
    std::cout << "test4-4" << std::endl;
    prog_shutdown = &dc_.tc_shm_->shutdown;
    std::cout << "test5" << std::endl;
    // ROS_INFO("!!!!!!!");

    bool zp_load;
    dc_.nh.param("/dyros_bolt_controller/force_load_zp", zp_load, false);

    dc_.tc_shm_->force_load_saved_signal = zp_load;

    // double Hz;
    // nh.param<double>("control_frequency", Hz, 200.0);
    std::string sysLogFile = "/home/dyros/dyros_bolt_log/system_report";

    ofstream system_log;

    if (dc_.tc_shm_->shutdown){

        std::cout << cred << "Shared memory was not successfully removed from the previous run. " << std::endl;
        std::cout << "Please Execute shm_reset : rosrun dyros_bolt_controller shm_reset " << std::endl;
        std::cout << "Or you can remove reset shared memory with 'sudo ipcrm -m " << shm_id_ << "'" << creset << std::endl;
    }
    else{

        const int thread_number = 4;


        struct sched_param param_st;
        struct sched_param param;
        struct sched_param param_controller;
        struct sched_param param_logger;
        pthread_attr_t attrs[thread_number];
        pthread_t threads[thread_number];
        param.sched_priority = 42 + 50;
        param_logger.sched_priority = 41 + 50;
        param_controller.sched_priority = 45 + 50;
        param_st.sched_priority = 45 + 50;
        cpu_set_t cpusets[thread_number];
        
        pthread_t loggerThread;
        pthread_attr_t loggerattrs;

        if (dc_.simMode){

            std::cout << "Simulation Mode" << std::endl;
        }
        if (!dc_.simMode){

            std::cout << "Real Robot Mode" << std::endl;
        }

        if (!dc_.simMode)
        {

            system_log.open(sysLogFile.c_str(), fstream::out | fstream::app);

            std::time_t t_clock_start = std::time(0);
            system_log << "===================================================================================================" << std::endl;
            system_log << "System Successfully Started " << std::ctime(&t_clock_start);

            if (pthread_attr_setschedparam(&attrs[0], &param_st))
            {
                printf("attr %d setschedparam failed ", 0);
            }
            
            if (pthread_attr_setschedparam(&attrs[1], &param_controller))
            {
                printf("attr %d setschedparam failed ", 0);
            }

            CPU_ZERO(&cpusets[0]);
            CPU_SET(5, &cpusets[0]);
            if (pthread_attr_setaffinity_np(&attrs[0], sizeof(cpu_set_t), &cpusets[0]))
            {
                printf("attr %d setaffinity failed ", 0);
            }

            if (pthread_attr_setschedpolicy(&loggerattrs, SCHED_FIFO))
            {
                printf("attr logger setschedpolicy failed ");
            }
            if (pthread_attr_setschedparam(&loggerattrs, &param_logger))
            {
                printf("attr logger setschedparam failed ");
            }
            if (pthread_attr_setinheritsched(&loggerattrs, PTHREAD_EXPLICIT_SCHED))
            {
                printf("attr logger setinheritsched failed ");
            }
        }

        if (pthread_create(&threads[0], &attrs[0], &StateManager::ThreadStarter, &stm)){
            
            printf("threads[0] create failed\n");
        }
        if (pthread_create(&threads[1], &attrs[1], &DyrosBoltController::Thread1Starter, &tc_)){
            
            printf("threads[1] create failed\n");
        }
        if (pthread_create(&threads[2], &attrs[2], &DyrosBoltController::Thread2Starter, &tc_)){
            
            printf("threads[2] create failed\n");
        }
        if (pthread_create(&threads[3], &attrs[3], &DyrosBoltController::Thread3Starter, &tc_)){
            
            printf("threads[3] create failed\n");
        }
        if (true)
        {

            if (pthread_create(&loggerThread, &loggerattrs, &StateManager::LoggerStarter, &stm))
            {
                printf("logger Thread create failed\n");
            }
        }
        for (int i = 0; i < thread_number; i++){
            
            pthread_attr_destroy(&attrs[i]);
        }

        for (int i = 0; i < thread_number; i++){
            
            pthread_join(threads[i], NULL);
        }

    }


    // if(mode == "simulation")
    // {
    //     ROS_INFO("DYROS BOLT MAIN CONTROLLER - !!! MUJOCO SIMULATION MODE !!!");
    //     ctr_obj = new mujoco_interface(nh, Hz);
    // }
    // else if(mode == "real_robot")
    // {
    //     ROS_INFO("DYROS BOLT MAIN CONTROLLER - !!! REAL ROBOT MODE !!!");
    //     ctr_obj = new RealRobotInterface(nh, Hz);
    // }
    // else
    // {
    //     ROS_FATAL("Please choose simulation or real_robot");
    // }

    // while(ros::ok())
    // {
    //     ctr_obj->readDevice();
    //     ctr_obj->update();
    //     ctr_obj->compute();
    //     ctr_obj->reflect();
    //     ctr_obj->writeDevice();
    //     ctr_obj->wait();

    //     if(ctr_obj->isShuttingDown())
    //     {
    //       break;
    //     }
    // }

    // delete ctr_obj;
    if (system_log.is_open())
    {
        std::time_t t_clock_end = std::time(0);

        int max_us = 30;

        int h_1 = 4;
        int h_2 = 10;

        int setw_up[max_us];
        int setw_low[max_us];

        for (int i = 0; i < max_us; i++)
        {
            int j = 0;
            while (pow(10, j) < dc_.tc_shm_->lat_h[i])
            {
                j++;
            }
            while (pow(10, j) < dc_.tc_shm_->rcv_h[i])
            {
                j++;
            }
            while (pow(10, j) < dc_.tc_shm_->send_h[i])
            {
                j++;
            }
            while (pow(10, j) < dc_.tc_shm_->lat2_h[i])
            {
                j++;
            }
            while (pow(10, j) < dc_.tc_shm_->rcv2_h[i])
            {
                j++;
            }
            while (pow(10, j) < dc_.tc_shm_->send2_h[i])
            {
                j++;
            }

            if (j < 2)
                j = 2;

            setw_up[i] = j;
        }

        system_log << "System Successfully Ended   " << std::ctime(&t_clock_end);
        int hr, mr;
        double sr;

        hr = dc_.rd_.control_time_ / 3600;
        mr = (dc_.rd_.control_time_ - hr * 3600) / 60;

        sr = dc_.rd_.control_time_ - hr * 3600 - mr * 60;

        if (hr > 0)
        {
            system_log << "Running Time : " << hr << " h : " << mr << " m : " << fixed << setprecision(3) << sr << " s" << std::endl;
        }
        else if (mr > 0)
        {
            system_log << "Running Time : " << mr << " m : " << fixed << setprecision(3) << sr << " s" << std::endl;
        }
        else
        {
            system_log << "Running Time : " << fixed << setprecision(3) << dc_.rd_.control_time_ << " s" << std::endl;
        }
        system_log << "ECAT UPPER REPORT | TOTAL COUNT : " << dc_.tc_shm_->statusCount << std::endl;
        system_log << "Latency    avg : " << fixed << setprecision(3) << setw(6) << dc_.tc_shm_->lat_avg / 1000.0 << " max : " << fixed << setprecision(3) << setw(6) << dc_.tc_shm_->lat_max / 1000.0 << std::endl;
        system_log << "ec_receive avg : " << fixed << setprecision(3) << setw(6) << dc_.tc_shm_->send_avg / 1000.0 << " max : " << fixed << setprecision(3) << setw(6) << dc_.tc_shm_->send_max / 1000.0 << " ovf : " << dc_.tc_shm_->send_ovf << std::endl;
        system_log << "ec_send    avg : " << fixed << setprecision(3) << setw(6) << dc_.tc_shm_->rcv_avg / 1000.0 << " max : " << fixed << setprecision(3) << setw(6) << dc_.tc_shm_->rcv_max / 1000.0 << std::endl;
        system_log << "Histogram : ";
        for (int i = 0; i < 29; i++)
            system_log << std::setfill(' ') << std::setw(setw_up[i]) << i << "  ";
        system_log << " +";
        system_log << std::endl;
        system_log << "latency   : ";
        for (int i = 0; i < 30; i++)
            system_log << std::setfill(' ') << std::setw(setw_up[i]) << dc_.tc_shm_->lat_h[i] << "  ";
        system_log << std::endl;
        system_log << "ec_recv   : ";
        for (int i = 0; i < 30; i++)
            system_log << std::setfill(' ') << std::setw(setw_up[i]) << dc_.tc_shm_->rcv_h[i] << "  ";
        system_log << std::endl;
        system_log << "ec_send   : ";
        for (int i = 0; i < 30; i++)
            system_log << std::setfill(' ') << std::setw(setw_up[i]) << dc_.tc_shm_->send_h[i] << "  ";
        system_log << std::endl;

        system_log << "ECAT LOWER REPORT | TOTAL COUNT : " << dc_.tc_shm_->statusCount2 << std::endl;
        system_log << "Latency    avg : " << fixed << setprecision(3) << setw(6) << dc_.tc_shm_->lat_avg2 / 1000.0 << " max : " << fixed << setprecision(3) << setw(6) << dc_.tc_shm_->lat_max2 / 1000.0 << std::endl;
        system_log << "ec_receive avg : " << fixed << setprecision(3) << setw(6) << dc_.tc_shm_->send_avg2 / 1000.0 << " max : " << fixed << setprecision(3) << setw(6) << dc_.tc_shm_->send_max2 / 1000.0 << " ovf : " << dc_.tc_shm_->send_ovf2 << std::endl;
        system_log << "ec_send    avg : " << fixed << setprecision(3) << setw(6) << dc_.tc_shm_->rcv_avg2 / 1000.0 << " max : " << fixed << setprecision(3) << setw(6) << dc_.tc_shm_->rcv_max2 / 1000.0 << std::endl;
        system_log << "Histogram : ";
        for (int i = 0; i < 29; i++)
            system_log << std::setfill(' ') << std::setw(setw_up[i]) << i << "  ";
        system_log << " +";
        system_log << std::endl;
        system_log << "latency   : ";
        for (int i = 0; i < 30; i++)
            system_log << std::setfill(' ') << std::setw(setw_up[i]) << dc_.tc_shm_->lat2_h[i] << "  ";
        system_log << std::endl;
        system_log << "ec_recv   : ";
        for (int i = 0; i < 30; i++)
            system_log << std::setfill(' ') << std::setw(setw_up[i]) << dc_.tc_shm_->rcv2_h[i] << "  ";
        system_log << std::endl;
        system_log << "ec_send   : ";
        for (int i = 0; i < 30; i++)
            system_log << std::setfill(' ') << std::setw(setw_up[i]) << dc_.tc_shm_->send2_h[i] << "  ";
        system_log << std::endl
                   << std::endl;

        system_log.close();
    }


    ros::shutdown();
    deleteSharedMemory(shm_id_, dc_.tc_shm_);
    std::cout << cgreen << " CNTRL : Dyros Bolt Controller Shutdown" << creset << std::endl;


    return 0;
}


