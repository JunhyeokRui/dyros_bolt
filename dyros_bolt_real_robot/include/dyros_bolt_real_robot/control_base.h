#ifndef _CONTROL_BASE_H
#define _CONTROL_BASE_H

// STD Library
#include <array>
#include <vector>
#include <iostream>
#include <fstream>

// System Library
#include <termios.h>

// ROS Library
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>

// ROS Messages
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/server/simple_action_server.h>

#include <dyros_bolt_msgs/JointSet.h>
#include <dyros_bolt_msgs/JointState.h>
#include <dyros_bolt_msgs/TaskCommand.h>
#include <dyros_bolt_msgs/JointCommand.h>
#include <dyros_bolt_msgs/WalkingCommand.h>
#include <dyros_bolt_msgs/CustomCommand.h>
#include <dyros_bolt_msgs/WalkingState.h>
#include <dyros_bolt_msgs/JointControlAction.h>

// User Library
#include "math_type_define.h"
#include "dyros_bolt_lib/robot_data.h"
#include "dyros_bolt_lib/state_manager.h"

// #include "dyros_bolt_controller/controller.h"
// #include "dyros_bolt_controller/joint_controller.h"
// #include "dyros_bolt_controller/custom_controller.h"
// #include "dyros_bolt_controller/rl_controller.h"
// #include "dyros_bolt_controller/dyros_bolt_model.h"
#include "shm_msgs.h"


#define MODEL_DOF 6


namespace dyros_bolt_real_robot
{

using namespace Eigen;
using namespace std;
using namespace DYROS_BOLT;

class ControlBase
{

public:
    ControlBase(ros::NodeHandle &nh, double Hz);
    virtual ~ControlBase(){}
    // Default User Call function
    void parameterInitialize(); // initialize all parameter function(q,qdot,force else...)
    virtual void readDevice(); // read device means update all subscribed sensor data and user command
    virtual void update(); // update controller based on readdevice
    virtual void compute(); // compute algorithm and update all class object
    virtual void reflect(); // reflect next step actuation such as motor angle else
    virtual void writeDevice()=0; // publish to actuate devices
    virtual void wait()=0;  // wait
    bool isShuttingDown() const {return shutdown_flag_;}

    double currentTime(){return control_time_;};
    void syncSimControlTime(double time){control_time_ = time; std::cout <<"control time "<< (double)control_time_ <<std::endl;};

    const double getHz() { return Hz_; }

    std::vector<double> pos_kp;
    std::vector<double> pos_kv;
    std::vector<double> k_tau;

private:
    void makeIDInverseList();

    double Hz_; ///< control
    unsigned long tick_;
    double control_time_;

    bool shutdown_flag_;

    ros::Subscriber joint_command_sub_;
    ros::Subscriber custom_command_sub_;
    ros::Subscriber shutdown_command_sub_;
    ros::Subscriber rl_command_sub_;

    dyros_bolt_msgs::JointControlFeedback joint_control_feedback_;
    dyros_bolt_msgs::JointControlResult joint_control_result_;
    actionlib::SimpleActionServer<dyros_bolt_msgs::JointControlAction>  joint_control_as_;  // Action Server

    void jointCommandCallback(const dyros_bolt_msgs::JointCommandConstPtr& msg);
    void customCommandCallback(const dyros_bolt_msgs::CustomCommandConstPtr& msg);
    void shutdownCommandCallback(const std_msgs::StringConstPtr& msg);
    // void jointControlActionCallback(const dyros_bolt_msgs::JointControlGoalConstPtr &goal);   
    void rlCommandCallback(const std_msgs::BoolConstPtr& msg); 

protected:
    realtime_tools::RealtimePublisher<dyros_bolt_msgs::JointState> joint_state_pub_; 
    realtime_tools::RealtimePublisher<sensor_msgs::JointState> joint_robot_state_pub_;

    unsigned int joint_id_[MODEL_DOF];
    unsigned int joint_id_inversed_[ MODEL_DOF];
    unsigned int control_mask_[ MODEL_DOF];

    bool is_first_boot_;
    bool extencoder_init_flag_;

    VectorQd q_; ///< current q
    VectorQd q_dot_; ///< current qdot
    VectorQd q_dot_filtered_; ///< current qdot with filter
    VectorQd torque_; // current joint toruqe
    Eigen::Vector12d q_ext_;
    Eigen::Vector12d q_ext_dot_;
    Eigen::Vector12d q_ext_offset_;

    Vector6d left_foot_ft_; // current left ft sensor values
    Vector6d right_foot_ft_; // current right ft sensor values

    tf::Quaternion imu_data_; ///< IMU data with filter
    Eigen::Quaterniond base_quat_; ///< IMU data without filter
    Vector4d imu_data_quat; 
    Vector3d gyro_; // current gyro sensor values
    Vector3d accelometer_; // current accelometer values
    Vector3d imu_accelometer_; // current accelometer values
    Vector3d imu_angular_velocity;
    Vector3d imu_grav_rpy_;
    Matrix3d pelvis_orientation_;

    Vector3d com_sim_; //com position from simulation COMvisualziefunction
    Eigen::Isometry3d lfoot_global_;
    Eigen::Isometry3d rfoot_global_;
    Eigen::Isometry3d base_global_;
    
    VectorQd desired_q_; // current desired joint values
    VectorQd desired_torque_; // current desired torque values
    VectorQf desired_torque_f; // current desired torque values
    Eigen::Vector12d extencoder_offset_;

    Eigen::Vector6d mujoco_virtual_;
    Eigen::Vector6d mujoco_virtual_dot_;


    int total_dof_;
    


};

}
#endif