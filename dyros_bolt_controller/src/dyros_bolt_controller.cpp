// #include <ros/ros.h>
// #include "dyros_bolt_controller/mujoco_interface.h"
// #include "dyros_bolt_controller/real_robot_interface.h"
// using namespace dyros_bolt_controller;

#include <math.h>

// volatile bool *prog_shutdown;

// void SIGINT_handler(int sig)
// {
//     cout << " CNTRL : shutdown Signal" << endl;
//     *prog_shutdown = true;
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "dyros_bolt_controller");
//     ros::NodeHandle nh("~");

//     std::string mode;
//     nh.param<std::string>("run_mode", mode, "simulation");
//     ControlBase *ctr_obj;
//     ROS_INFO("!!!!!!!");
    
    // int shm_id_;
    // init_shm(shm_msg_key, shm_id_, &ctr_obj->tc_shm_);
    // prog_shutdown = &ctr_obj->tc_shm_->shutdown;

//     double Hz;
//     nh.param<double>("control_frequency", Hz, 200.0);

//     if(mode == "simulation")
//     {
//         ROS_INFO("DYROS BOLT MAIN CONTROLLER - !!! MUJOCO SIMULATION MODE !!!");
//         ctr_obj = new mujoco_interface(nh, Hz);
//     }
//     else if(mode == "real_robot")
//     {
//         ROS_INFO("DYROS BOLT MAIN CONTROLLER - !!! REAL ROBOT MODE !!!");
//         ctr_obj = new RealRobotInterface(nh, Hz);
//     }
//     else
//     {
//         ROS_FATAL("Please choose simulation or real_robot");
//     }

//     while(ros::ok())
//     {
//         ctr_obj->readDevice();
//         ctr_obj->update();
//         ctr_obj->compute();
//         ctr_obj->reflect();
//         ctr_obj->writeDevice();
//         ctr_obj->wait();

//         if(ctr_obj->isShuttingDown())
//         {
//           break;
//         }
//     }

//     delete ctr_obj;

//     return 0;
// }


#include "dyros_bolt_controller/dyros_bolt_controller.h"

using namespace std;

DyrosBoltController::DyrosBoltController(StateManager &stm_global) :  dc_(stm_global.dc_), stm_(stm_global), rd_(stm_global.dc_.rd_)
#ifdef COMPILE_DYROS_BOLT_CC
                                                               ,
                                                               my_cc(*(new CustomController(rd_, stm_, dc_)))
#endif
{   
    nh_controller_.setCallbackQueue(&queue_controller_);

    task_command_sub_ = nh_controller_.subscribe("/dyros_bolt/taskcommand", 100, &DyrosBoltController::TaskCommandCallback, this);
    task_command_que_sub_ = nh_controller_.subscribe("/dyros_bolt/taskquecommand", 100, &DyrosBoltController::TaskQueCommandCallback, this);
    position_command_sub_ = nh_controller_.subscribe("/dyros_bolt/positioncommand", 100, &DyrosBoltController::PositionCommandCallback, this);
    task_gain_sub_ = nh_controller_.subscribe("/dyros_bolt/taskgaincommand", 100, &DyrosBoltController::TaskGainCommandCallback, this);

    ros::param::get("/dyros_bolt_controller/Kp", rd_.pos_kp_v);
    ros::param::get("/dyros_bolt_controller/Kv", rd_.pos_kv_v);
}

DyrosBoltController::~DyrosBoltController()
{
    cout << "DyrosBoltController Terminated" << endl;
}

void *DyrosBoltController::Thread1()
{
    volatile int rcv_time_ = 0;

    while (!rd_.firstCalc)
    {
        std::this_thread::sleep_for(std::chrono::microseconds(1));
        if (dc_.tc_shm_->shutdown)
        {
            break;
        }
    }

    std::cout << " Thread1 Proceeding ... " << std::endl;

    EnableThread2(true); // Set true for Thread2
    EnableThread3(true); // True for thread3 ...

    if (dc_.simMode)
    {
        for (int i = 0; i < LINK_NUMBER + 1; i++)
        {
            rd_.link_[i].pos_p_gain << 400, 400, 400;
            rd_.link_[i].pos_d_gain << 40, 40, 40;
            rd_.link_[i].pos_a_gain << 1, 1, 1;

            rd_.link_[i].rot_p_gain << 400, 400, 400;
            rd_.link_[i].rot_d_gain << 40, 40, 40;
            rd_.link_[i].rot_a_gain << 1, 1, 1;
        }
    }
    else
    {
        for (int i = 0; i < LINK_NUMBER + 1; i++)
        {
            rd_.link_[i].pos_p_gain << 80, 80, 80;
            rd_.link_[i].pos_d_gain << 10, 10, 10;
            rd_.link_[i].pos_a_gain << 1, 1, 1;

            rd_.link_[i].rot_p_gain << 200, 200, 200;
            rd_.link_[i].rot_d_gain << 16, 16, 16;
            rd_.link_[i].rot_a_gain << 1, 1, 1;
        }
        std::cout << " CNTRL : Set with realrobot gain" << std::endl;
    }

    signalThread1 = true;
    int thread1_count = 0;
    while (!dc_.tc_shm_->shutdown) //ANCHOR - while not shutdown
    { 
        if (dc_.triggerThread1)
        {
            dc_.triggerThread1 = false;

            thread1_count++;

            if (dc_.tc_shm_->shutdown)
                break;

            rcv_time_ = rd_.control_time_us_;

            auto t1 = std::chrono::steady_clock::now();

            queue_controller_.callAvailable(ros::WallDuration());
            //rui - Start Dyros Bolt Control
            //////////////////////////////////////////////////////////
            ////////////////////Start Dyros Bolt Controll/////////////////
            //////////////////////////////////////////////////////////

            VectorQd torque_task_, torque_grav_, torque_contact_;
            torque_task_.setZero();
            torque_grav_.setZero();
            torque_contact_.setZero();
            static VectorQd zero_m = VectorQd::Zero();

            if (rd_.positionControlSwitch) // if position control switch
            { 
            
                rd_.positionControlSwitch = false;

                rd_.q_desired = rd_.q_;
                rd_.positionHoldSwitch = true;
                rd_.pc_mode = true;

                std::cout << " CNTRL : Position Hold!" << rd_.control_time_ << std::endl;

            }

            if (rd_.pc_mode) //ANCHOR - position control mode
            { 

                if (rd_.positionHoldSwitch) 
                {}
                else 
                {

                    rd_.q_desired = DyrosMath::cubicVector(rd_.control_time_, rd_.pc_time_, rd_.pc_time_ + rd_.pc_traj_time_, rd_.pc_pos_init, rd_.pc_pos_des, rd_.pc_vel_init, zero_m);
                }

                for (int i = 0; i < MODEL_DOF; i++) 
                {

                    rd_.torque_desired[i] = rd_.pos_kp_v[i] * (rd_.q_desired[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * (zero_m[i] - rd_.q_dot_[i]);
                }
            }
            else if (rd_.tc_run)  //ANCHOR - torque control run
            { 

                if (rd_.tc_.mode == 0) 
                {
                    if (rd_.tc_init)
                    {

                        std::cout << "mode 0 init" << std::endl;
                        rd_.tc_init = false;

                        rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;
                    }

                }
//rui - Dyros_Bolt_CC [tc_.mode 6, 7, 8]
#ifdef COMPILE_DYROS_BOLT_CC
                if ((rd_.tc_.mode > 5) && (rd_.tc_.mode < 9)) // 6,7,8
                {
                        my_cc.computeSlow();
                }
#endif
            }
            else
            {
                // WBC::SetContact(rd_, 1, 1);
                // rd_.torque_desired = WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_));
            }


            static std::chrono::steady_clock::time_point t_c_ = std::chrono::steady_clock::now();


            SendCommand(rd_.torque_desired);

            auto t_end = std::chrono::steady_clock::now();

            auto d1 = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t1).count();            // 150us without march=native
            auto d2 = std::chrono::duration_cast<std::chrono::microseconds>(t_end - rd_.tp_state_).count(); // 150us without march=native

            static int d1_over_cnt = 0;

            if (d1 > 500) 
            {

                d1_over_cnt++;
            }

            static int d2_total = 0;
            static double d1_total = 0;

            d2_total += d2;
            d1_total += d1;

            dc_.tcm_cnt = thread1_count;
            if (thread1_count % 2000 == 0) 
            {

                if (d1_over_cnt > 0) 
                {

                    std::cout << cred << " DYNCS : Thread1 calculation time over 500us.. : " << d1_over_cnt << "times, stm cnt : " << dc_.tc_shm_->stloopCount << creset << std::endl;
                    d1_over_cnt = 0;
                }

                d1_total = 0;
                d2_total = 0;
                rd_.state_ctime_total_ = 0;
            }
            t_c_ = std::chrono::steady_clock::now();

        }
        else 
        {

            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
    }

    return (void *)NULL;
}    

void *DyrosBoltController::Thread2()
{

    std::cout << " CNTRL2 : started with pid : " << getpid() << std::endl;
    
    while (true) 
    {

        if (signalThread1 || dc_.tc_shm_->shutdown)
            break;
            
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
    if (enableThread2) 
    {

        std::cout << " CNTRL : Thread2_entered" << std::endl;
        while (!dc_.tc_shm_->shutdown) 
        {
            
            if (triggerThread2) 
            {

                triggerThread2 = false;
                
                /////////////////////////////////////////////
                /////////////Do something in Thread2 !!!!!!!
                /////////////////////////////////////////////

                if (rd_.tc_run) 
                {

#ifdef COMPILE_DYROS_BOLT_CC
                    if ((rd_.tc_.mode > 5) && (rd_.tc_.mode < 9)) // 6,7,8
                    {
                        my_cc.computeFast();
                    }
#endif
                }
                std::this_thread::sleep_for(std::chrono::microseconds(10));
            }
            else {

                std::this_thread::sleep_for(std::chrono::microseconds(10));
            }
        }
    }
    else 
    {
        std::cout << " CNTRL : Thread2 disabled" << std::endl;
    }

    std::cout << " CNTRL : Thread2 terminate" << std::endl;
    return (void *)NULL;
}

void *DyrosBoltController::Thread3()
{

    while (true) 
    {

        if (signalThread1 || dc_.tc_shm_->shutdown)
            break;

        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
    if (enableThread3) 
    {

        std::cout << " CNTRL : Thread3_entered" << std::endl;

        while (!dc_.tc_shm_->shutdown) 
        {
            if (triggerThread3) 
            {

                triggerThread3 = false;
                /////////////////////////////////////////////
                /////////////Do something in Thread3 !!!!!!!
                /////////////////////////////////////////////


            }
            else 
            {

                std::this_thread::sleep_for(std::chrono::microseconds(1));
            }
        }
    }
    else 
    {

        std::cout << " CNTRL : Thread3 disabled" << std::endl;
    }

    std::cout << " CNTRL : Thread3 terminate" << std::endl;
    return (void *)NULL;

}

void DyrosBoltController::MeasureTime(int currentCount, int nanoseconds1, int nanoseconds2)
{
    dc_.tc_shm_->t_cnt2 = currentCount;

    lat = nanoseconds1;
    total1 += lat;
    lavg = total1 / currentCount;
    if (lmax < lat)
    {
        lmax = lat;
    }
    if (lmin > lat)
    {
        lmin = lat;
    }
    // int sdev = (sat - savg)
    total_dev1 += sqrt(((lat - lavg) * (lat - lavg)));
    ldev = total_dev1 / currentCount;

    dc_.tc_shm_->lat_avg2 = lavg;
    dc_.tc_shm_->lat_max2 = lmax;
    dc_.tc_shm_->lat_min2 = lmin;
    dc_.tc_shm_->lat_dev2 = ldev;

    sat = nanoseconds2;
    total2 += sat;
    savg = total2 / currentCount;
    if (smax < sat)
    {
        smax = sat;
    }
    if (smin > sat)
    {
        smin = sat;
    }
    // int sdev = (sat - savg)
    total_dev2 += sqrt(((sat - savg) * (sat - savg)));
    sdev = total_dev2 / currentCount;

    dc_.tc_shm_->send_avg2 = savg;
    dc_.tc_shm_->send_max2 = smax;
    dc_.tc_shm_->send_min2 = smin;
    dc_.tc_shm_->send_dev2 = sdev;
}

void DyrosBoltController::SendCommand(Eigen::VectorQd torque_command)
{
    while (dc_.t_c_)
    {
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
    dc_.t_c_ = true;
    dc_.control_command_count++;
    std::copy(torque_command.data(), torque_command.data() + MODEL_DOF, dc_.torque_command);
    dc_.t_c_ = false;
}

void DyrosBoltController::EnableThread2(bool enable)
{
    enableThread2 = enable;
}

void DyrosBoltController::EnableThread3(bool enable)
{
    enableThread3 = enable;
}

void DyrosBoltController::RequestThread2()
{
    triggerThread2 = true;
}

void DyrosBoltController::RequestThread3()
{
    triggerThread3 = true;
}

void DyrosBoltController::PositionCommandCallback(const dyros_bolt_msgs::positionCommandConstPtr &msg)
{
    static bool position_command = false;

    rd_.pc_traj_time_ = msg->traj_time;

    rd_.pc_time_ = rd_.control_time_;

    rd_.pc_pos_init = rd_.q_;
    rd_.pc_vel_init = rd_.q_dot_;

    if (position_command && msg->relative) 
    {
        
        rd_.pc_pos_init = rd_.q_desired;
        std::cout << " CNTRL : pos init with prev des" << std::endl;
    }

    for (int i = 0; i < MODEL_DOF; i++) 
    {
        rd_.pc_pos_des(i) = msg->position[i];
    }
    rd_.pc_mode = true;
    rd_.pc_gravity = msg->gravity;
    rd_.positionHoldSwitch = false;

    stm_.StatusPub("%f Position Control", (float)rd_.control_time_);
    position_command = true;
    std::cout << " CNTRL : Position command received" << std::endl;
}

void DyrosBoltController::TaskCommandCallback(const dyros_bolt_msgs::TaskCommandConstPtr &msg)
{
    rd_.pc_mode = false;
    rd_.tc_ = *msg;
    std::cout << " CNTRL : task signal received mode :" << rd_.tc_.mode << std::endl;
    std::cout << "test1" << std::endl;
    stm_.StatusPub("%f task Control mode : %d", (float)rd_.control_time_, rd_.tc_.mode);
    std::cout << "test2" << std::endl;
    rd_.tc_time_ = rd_.control_time_;
    rd_.tc_run = true;
    rd_.tc_init = true;
    rd_.link_[FR_LOWER_LEG].SetInitialWithPosition();
    rd_.link_[FL_LOWER_LEG].SetInitialWithPosition();
    rd_.link_[base_link].SetInitialWithPosition();
    rd_.link_[COM_id].SetInitialWithPosition();

    std::cout << "test3" << std::endl;
    if (!rd_.semode)
    {
        std::cout << " CNTRL : State Estimate is not running. disable task command" << std::endl;
        rd_.tc_run = false;
    }
}


void DyrosBoltController::TaskGainCommandCallback(const dyros_bolt_msgs::TaskGainCommandConstPtr &msg)
{
    int __id = 0;
    bool okcheck = false;
    if (msg->mode == 1)
    {
        __id = COM_id;
    }
    else if (msg->mode == 2)
    {
        __id = base_link;
    }
    else if (msg->mode == 5)
    {

        __id = FR_LOWER_LEG;
        std::cout << "Gain set on FR_LOWER_LEG" << std::endl;
    }
    else if (msg->mode == 6)
    {
        __id = FL_LOWER_LEG;
        std::cout << "Gain set on FL_LOWER_LEG" << std::endl;
    }
    else
    {
        okcheck = true;
    }

    if (okcheck)
    {
        std::cout << "Wrong Mode " << std::endl;
    }
    else
    {
        rd_.link_[__id].pos_p_gain[0] = msg->pgain[0];
        rd_.link_[__id].pos_p_gain[1] = msg->pgain[1];
        rd_.link_[__id].pos_p_gain[2] = msg->pgain[2];

        rd_.link_[__id].pos_d_gain[0] = msg->dgain[0];
        rd_.link_[__id].pos_d_gain[1] = msg->dgain[1];
        rd_.link_[__id].pos_d_gain[2] = msg->dgain[2];

        rd_.link_[__id].rot_p_gain[0] = msg->pgain[3];
        rd_.link_[__id].rot_p_gain[1] = msg->pgain[4];
        rd_.link_[__id].rot_p_gain[2] = msg->pgain[5];

        rd_.link_[__id].rot_d_gain[0] = msg->dgain[3];
        rd_.link_[__id].rot_d_gain[1] = msg->dgain[4];
        rd_.link_[__id].rot_d_gain[2] = msg->dgain[5];

        std::cout << "Gain Set " << rd_.link_[__id].id << "  POS p : " << rd_.link_[__id].pos_p_gain.transpose() << "   d :" << rd_.link_[__id].pos_d_gain.transpose() << std::endl;
        std::cout << "Gain Set " << rd_.link_[__id].id << "  ROT p : " << rd_.link_[__id].rot_p_gain.transpose() << "   d :" << rd_.link_[__id].rot_d_gain.transpose() << std::endl;
    }
}

void DyrosBoltController::TaskQueCommandCallback(const dyros_bolt_msgs::TaskCommandQueConstPtr &msg)
{
    rd_.tc_q_ = *msg;
    rd_.task_que_signal_ = true;
    std::cout << "task que received ... but doing nothing .." << std::endl;
}
