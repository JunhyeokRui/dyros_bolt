#include "dyros_bolt_real_robot/real_robot_interface.h"

volatile bool *prog_shutdown;
int shm_id_;
SHMmsgs *shm_msgs_;


void SIGINT_handler(int sig)
{
    std::cout << " CNTRL : shutdown Signal" << std::endl;
    *prog_shutdown = true;
}

namespace dyros_bolt_real_robot
{


RealRobotInterface::RealRobotInterface(ros::NodeHandle &nh, double Hz):
  ControlBase(nh, Hz), rate_(Hz), odrv(nh)
{
    ROS_INFO("ODrive starting up");

    // nh.param<std::string>("ctrl_mode", ctrl_mode, "torque");

    axis_request_state_sub = nh.subscribe<std_msgs::Int16>("/odrv_axis_request_states", 1, &RealRobotInterface::axisRequestStateCallback, this);
    axis_current_state_pub = nh.advertise<std_msgs::Int16MultiArray>("/odrv_axis_current_states", 1);

    init_shm(shm_msg_key, shm_id_, &tc_shm_);
    prog_shutdown = &tc_shm_->shutdown;
    ROS_INFO("TEST1");
}

void RealRobotInterface::axisRequestStateCallback(const std_msgs::Int16::ConstPtr& msg) {
    int16_t requestState = msg->data;
    
    switch (requestState) {
        case 1:
            odrv.disengage();
            break;
        case 2:
            for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
                odrv.requestODriveCmd(i, odrive::ODriveCommandId::ESTOP_MESSAGE);
            }
            break;    
        case 4:
            for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
                odrv.setAxisRequestedState(odrv.axis_can_ids_list[i], odrive::ODriveAxisState::MOTOR_CALIBRATION);
            }
            break;
        case 7:
            for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
                odrv.setAxisRequestedState(odrv.axis_can_ids_list[i], odrive::ODriveAxisState::ENCODER_OFFSET_CALIBRATION);
            }
            break;
        case 8:
            odrv.engage();
            // for(int i=0; i<  MODEL_DOF / 2 - 1; i++)
            // {
            //     // odrv.setInputTorque(i, 0);
            //     // odrv.setInputTorque(i+3,0);
            // }
            break;
        case 16:
            for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
                odrv.requestODriveCmd(i, odrive::ODriveCommandId::REBOOT_ODRIVE);
            }
            break;
        case 19:
            for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
                odrv.resetEncoder(i, odrive::ODriveCommandId::SET_ABSOLUTE_POSITION);
            }
            break;    
    }
}

void RealRobotInterface::readDevice()
{
    ControlBase::readDevice();
    axisCurrentPublish();
    
    for (int i = 0; i < 3; i++)
    {
        q_(i) = -odrv.axis_angle[i];
        q_dot_(i) = odrv.axis_velocity[i];

        q_(i+3) = odrv.axis_angle[i+3];
        q_dot_(i+3) = odrv.axis_velocity[i+3];
    }
    q_(3) = -q_(3);
    // q_[3] = 0;
    // q_[7] = 0;
    // q_dot_[3] =0;
    // q_dot_[7] =0;

    imu_data_quat(0) = tc_shm_->pos_virtual[0];
    imu_data_quat(1) = tc_shm_->pos_virtual[1];
    imu_data_quat(2) = tc_shm_->pos_virtual[2];
    imu_data_quat(3) = tc_shm_->pos_virtual[3];

    imu_accelometer_(0) = tc_shm_->imu_acc[0];
    imu_accelometer_(1) = tc_shm_->imu_acc[1];
    imu_accelometer_(2) = tc_shm_->imu_acc[2];

    imu_angular_velocity(0) = tc_shm_->vel_virtual[0];
    imu_angular_velocity(1) = tc_shm_->vel_virtual[1];
    imu_angular_velocity(2) = tc_shm_->vel_virtual[2];

    // std::cout << "imu_data_: " << std::endl;
    // std::cout << imu_data_ << std::endl << std::endl;

    // std::cout << "acc: " << std::endl;
    // std::cout << imu_accelometer_ << std::endl << std::endl;

    // std::cout << "angular_vel: " << std::endl;
    // std::cout << imu_angular_velocity << std::endl << std::endl;
    std::cout << "q_" << std::endl;
    for (size_t i = 0; i < 6; i++)
    {
        std::cout << q_(i) << std::endl;
        
    }
    std::cout << "q_dot_" << std::endl;
    for (size_t i = 0; i < 6; i++)
    {
        std::cout << q_dot_(i) << std::endl;
    }

}

void RealRobotInterface::update()
{
    ControlBase::update();

    std::copy(tc_shm_->torqueCommand, tc_shm_->torqueCommand + 6, torque_desired_);

    // Assuming torque_desired_ is an array or supports indexing and has a size of 6
    for(int i = 0; i < 6; ++i) {
        std::cout << "torque_desired_[" << i << "] = " << torque_desired_[i] << std::endl;
    }
}

void RealRobotInterface::writeDevice()
{
    if(areMotorsReady()){
        // if (ctrl_mode == "torque"){
        
        // for(int i=0; i<  MODEL_DOF / 2; i++)
        // {
        //     odrv.setInputTorque(i, float(-torque_desired_(i)/k_tau[i]));
        //     odrv.setInputTorque(i+3, float(torque_desired_(i+3)/k_tau[i+3]));

        // }
        // }
        // else if (ctrl_mode == "position"){
        //     for(int i=0; i<  MODEL_DOF / 2 - 1; i++)
        //     {
        //         odrv.setInputPosition(i, double(desired_q_(i)));
        //         odrv.setInputPosition(i+3, double(desired_q_(i+4)));
        //     }
        // }


    }
}

void RealRobotInterface::wait()
{
    rate_.sleep();
}

bool RealRobotInterface::areMotorsReady()
{
    for(int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
    // for(int i = 0; i < 1; i++) {
        if(odrv.axis_current_state[i] != 8) {
            return false;
        }
    }
    return true;
}

void RealRobotInterface::axisCurrentPublish()
{
    std_msgs::Int16MultiArray state_msgs;
    for (int i = 0; i < 6; i++)
    {
        state_msgs.data.push_back(odrv.axis_current_state[i]);
    }
    axis_current_state_pub.publish(state_msgs);
}

// void RealRobotInterface::cleanupDyrosBoltSystem()
// {
//     init_shm(shm_msg_key, shm_id_, &shm_msgs_);
// }
// void RealRobotInterface::initDyrosBoltSystem()
// {
//     deleteSharedMemory(shm_id_, shm_msgs_);
// }

// void RealRobotInterface::getJointTorque()
// {
    
//     timespec ts_us1;

//     ts_us1.tv_sec = 0;
//     ts_us1.tv_nsec = 1000;

    
//     std::cout <<"test comupte0" << std::endl;
//     std::cout << "shm_msgs_->cmd_lower : " << shm_msgs_->cmd_lower << std::endl;
//     while (shm_msgs_->cmd_lower)
//     {
//         std::cout <<"test comupte0-1" << std::endl;
//         clock_nanosleep(CLOCK_MONOTONIC, 0, &ts_us1, NULL);
//     };
//     std::cout <<"test comupte1" << std::endl;
//     shm_msgs_->cmd_lower = true;
//     std::cout <<"test comupte2" << std::endl;
//     memcpy(&torque_desired_[0], &shm_msgs_->torqueCommand[0], sizeof(float) * MODEL_DOF);

//     shm_msgs_->cmd_lower = false;

//     std::cout <<"test comupte3" << std::endl;
// }


}