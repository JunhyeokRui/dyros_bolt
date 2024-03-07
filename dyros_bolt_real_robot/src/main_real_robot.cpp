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
#include "sys/mman.h"
#include <signal.h>


void SIGINT_handler(int sig)
{
    shutdownSystem();
}

int main(int argc, char **argv)
{
    mlockall(MCL_CURRENT | MCL_FUTURE);
    signal(SIGINT, SIGINT_handler);

    TocabiInitArgs init_args;

    struct sched_param param, param2;
    pthread_attr_t attr, attr2;
    pthread_t thread1, thread2, thread3;
    int ret;

    initDyrosBoltArgs(init_args);

    
    bool init_result = initDyrosBoltSystem(init_args);
    if (!init_result)
    {
        printf("[ECAT - ERRO] init failed\n");
        return -1;
    }

    
    ret = pthread_create(&thread1, &attr, ethercatThread1, &init_args);
    if (ret)
    {
        printf("create pthread 1 failed\n");
        return ret;
    }


    printf("[ECAT - INFO] cleaning up\n");
    cleanupDyrosBoltSystem();
    return 0;
}
