#include <ros/ros.h>
#include "dyros_bolt_real_robot/odrive_socketcan.h"
#include "dyros_bolt_real_robot/control_base.h"
#include "dyros_bolt_real_robot/real_robot_interface.h"

#include <iostream>
#include <math.h>

using namespace dyros_bolt_real_robot;

// #define stack64k (64 * 1024)
int main(int argc, char **argv)
{   
    ros::init(argc, argv, "dyros_bolt_real_robot");
    ros::NodeHandle nh("~");
    ControlBase *ctr_obj;

    // int shm_id_;
    // init_shm(shm_msg_key, shm_id_, &ctr_obj->tc_shm_);
    // prog_shutdown = &ctr_obj->tc_shm_->shutdown;

    double Hz;
    nh.param<double>("control_frequency", Hz, 150.0);

    ctr_obj = new RealRobotInterface(nh, Hz);

    while(ros::ok())
    {
        ctr_obj->readDevice();
        ctr_obj->update();
        // ctr_obj->compute();
        // ctr_obj->reflect();
        // ctr_obj->writeDevice();
        // ctr_obj->wait();

        // if(ctr_obj->isShuttingDown())
        // {
        //   break;
        // }
    }
    // delete ctr_obj;


    return 0;
}