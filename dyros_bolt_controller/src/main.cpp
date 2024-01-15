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
    ros::init(argc, argv, "dyros_bolt_controller", ros::init_options::NoSigintHandler);
    // ros::NodeHandle nh("~");

    DataContainer dc_;
    

    // std::string mode;

    dc_.nh.param("/dyros_bolt_controller/sim_mode", dc_.simMode, false);

    // nh.param<std::string>("run_mode", mode, "simulation");

    // ControlBase *ctr_obj;
    StateManager stm(dc_); //rui -
    DyrosBoltController tc_(stm); //rui
    
    int shm_id_;

    init_shm(shm_msg_key, shm_id_, &dc_.tc_shm_); //rui - init shared memory

    prog_shutdown = &dc_.tc_shm_->shutdown;

    // ROS_INFO("!!!!!!!");

    bool zp_load;
    dc_.nh.param("/dyros_bolt_controller/force_load_zp", zp_load, false);

    dc_.tc_shm_->force_load_saved_signal = zp_load;

    // double Hz;
    // nh.param<double>("control_frequency", Hz, 200.0);

    if (dc_.tc_shm_->shutdown){

        std::cout << cred << "Shared memory was not successfully removed from the previous run. " << std::endl;
        std::cout << "Please Execute shm_reset : rosrun dyros_bolt_controller shm_reset " << std::endl;
        std::cout << "Or you can remove reset shared memory with 'sudo ipcrm -m " << shm_id_ << "'" << creset << std::endl;
    }
    else{

        const int thread_number = 4;

        pthread_attr_t attrs[thread_number];
        pthread_t threads[thread_number];
        cpu_set_t cpusets[thread_number];

        if (dc_.simMode){

            std::cout << "Simulation Mode" << std::endl;
        }
        if (!dc_.simMode){

            std::cout << "Real Robot Mode" << std::endl;
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

    ros::shutdown();
    deleteSharedMemory(shm_id_, dc_.tc_shm_);
    std::cout << cgreen << " CNTRL : Dyros Bolt Controller Shutdown" << creset << std::endl;


    return 0;
}


