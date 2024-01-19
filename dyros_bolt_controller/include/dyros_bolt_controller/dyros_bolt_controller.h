#ifndef DYROS_BOLT_CONTROLLER_H
#define DYROS_BOLT_CONTROLLER_H

// #include "dyros_bolt_controller/state_manager.h"
#include "dyros_bolt_lib/state_manager.h"
#include "wholebody_functions.h"
#include "dyros_bolt_msgs/TaskGainCommand.h"

#ifdef COMPILE_DYROS_BOLT_CC
#include "cc.h"
#endif

class DyrosBoltController
{
public:
    // TocabiController(StateManager &stm_global);
    DyrosBoltController(StateManager &stm_global);

    // ~TocabiController();
    ~DyrosBoltController();
    void *Thread1();
    void *Thread2();
    void *Thread3();

    DataContainer &dc_;
    StateManager &stm_;
    RobotData &rd_;

#ifdef COMPILE_DYROS_BOLT_CC
    CustomController &my_cc;
#endif

    static void *Thread1Starter(void *context) { return ((DyrosBoltController *)context)->Thread1(); }
    static void *Thread2Starter(void *context) { return ((DyrosBoltController *)context)->Thread2(); }
    static void *Thread3Starter(void *context) { return ((DyrosBoltController *)context)->Thread3(); }

    void SendCommand(Eigen::VectorQd torque_command);
    void GetTaskCommand(dyros_bolt_msgs::TaskCommand &msg);

    void MeasureTime(int currentCount, int nanoseconds1, int nanoseconds2 = 0);
    int64_t total1 = 0, total2 = 0, total_dev1 = 0, total_dev2 = 0;
    float lmax = 0.0, lmin = 10000.00, ldev = 0.0, lavg = 0.0, lat = 0.0;
    float smax = 0.0, smin = 10000.00, sdev = 0.0, savg = 0.0, sat = 0.0;

    std::atomic<bool> enableThread2;
    void EnableThread2(bool enable);
    std::atomic<bool> enableThread3;
    void EnableThread3(bool enable);

    void RequestThread2();
    void RequestThread3();

    std::atomic<bool> signalThread1;
    std::atomic<bool> triggerThread2;
    std::atomic<bool> triggerThread3;

    ros::NodeHandle nh_controller_;
    ros::CallbackQueue queue_controller_;

    ros::Subscriber task_command_que_sub_;
    dyros_bolt_msgs::TaskCommandQue tc_que_msg_;
    
    ros::Subscriber task_command_sub_;
    dyros_bolt_msgs::TaskCommand tc_msg_;
    ros::Subscriber position_command_sub_;

    dyros_bolt_msgs::TaskGainCommand tcg_msg_;
    ros::Subscriber task_gain_sub_;

    void PositionCommandCallback(const dyros_bolt_msgs::positionCommandConstPtr &msg);
    void TaskCommandCallback(const dyros_bolt_msgs::TaskCommandConstPtr &msg);
    void TaskQueCommandCallback(const dyros_bolt_msgs::TaskCommandQueConstPtr &msg);
    void TaskGainCommandCallback(const dyros_bolt_msgs::TaskGainCommandConstPtr &msg);

};

#endif