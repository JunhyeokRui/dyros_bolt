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
odrive::ODriveSocketCan odrv;    

DyrosBoltInitArgs g_init_args;


bool initDyrosBoltArgs(const DyrosBoltInitArgs &args)
{
    
    
    Q_START = args.q_start_;
    PART_CAN_DOF = args.can_slave_num;

    extencoder_init_flag_ = false;

    init_shm(shm_msg_key, shm_id_, &shm_msgs_);
    // odrvInit();

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

    // while (!DyrosBoltInitialization)
    // {
    //     std::cout << " ODRV : START Initialization Mode" << std::endl;

    //     int16_t requestState = msg->data;
    
    //     switch (requestState) {
    //         case 1:
    //             odrv.disengage();
    //             break;
    //         case 2:
    //             for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
    //                 odrv.requestODriveCmd(i, odrive::ODriveCommandId::ESTOP_MESSAGE);
    //             }
    //             break;    
    //         case 4:
    //             for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
    //                 odrv.setAxisRequestedState(odrv.axis_can_ids_list[i], odrive::ODriveAxisState::MOTOR_CALIBRATION);
    //             }
    //             break;
    //         case 7:
    //             for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
    //                 odrv.setAxisRequestedState(odrv.axis_can_ids_list[i], odrive::ODriveAxisState::ENCODER_OFFSET_CALIBRATION);
    //             }
    //             break;
    //         case 8:
    //             odrv.engage();
    //             // for(int i=0; i< DyrosBoltModel::HW_TOTAL_DOF / 2 - 1; i++)
    //             // {
    //             //     // odrv.setInputTorque(i, 0);
    //             //     // odrv.setInputTorque(i+3,0);
    //             // }
    //             break;
    //         case 16:
    //             for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
    //                 odrv.requestODriveCmd(i, odrive::ODriveCommandId::REBOOT_ODRIVE);
    //             }
    //             break;
    //         case 19:
    //             for (int i = 0; i < odrv.axis_can_ids_list.size(); i++) {
    //                 odrv.resetEncoder(i, odrive::ODriveCommandId::SET_ABSOLUTE_POSITION);
    //             }
    //             break;    
    //     }
    //     if (DyrosBoltInitialization){
    //         break;
    //     }
    //     std::cout << " ODRV : Initialization Mode Failed. Retrying..." << std::endl;
    // }
    shm_msgs_->shutdown = true;
    if (shm_msgs_->shutdown)
        printf("Shutdown Command Before Start\n");

    while (!shm_msgs_->shutdown)
    {
        std::cout << "test2" << std::endl;
        if (!shm_msgs_->shutdown)
            printf(" REALROBOT : Control Mode Start ... ");

        while (!shm_msgs_->shutdown)
        {
            std::cout << "test3" << std::endl;
            readDevice();
            std::cout << "test3-1" << std::endl;
            sendJointStatus();
            std::cout << "test3-2" << std::endl;
            update();
            std::cout << "test3-3" << std::endl;
            getJointCommand();
            std::cout << "test3-4" << std::endl;
            // writeDevice();
        }


    }
    std::cout << "test4" << std::endl;
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

    std::cout << "q_" << std::endl;
    for (size_t i = 0; i < 6; i++)
    {
        std::cout << q_[i] << std::endl;
        
    }
    std::cout << "q_dot_" << std::endl;
    for (size_t i = 0; i < 6; i++)
    {
        std::cout << q_dot_[i] << std::endl;
    }

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
    std::cout << "test3-3-1" << std::endl;
    timespec ts_us1;

    ts_us1.tv_sec = 0;
    ts_us1.tv_nsec = 1000;
    std::cout << "test3-3-2" << std::endl;
    cpu_relax();
    std::cout << "test3-3-3" << std::endl;
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
        std::cout << "test3-3-4" << std::endl;
        clock_nanosleep(CLOCK_MONOTONIC, 0, &ts_us1, NULL);
    };
    shm_msgs_->cmd_lower = true;
    std::cout << "test3-3-5" << std::endl;
    memcpy(&torque_desired_[Q_START], &shm_msgs_->torqueCommand[Q_START], sizeof(float) * PART_CAN_DOF);
    for(int i = Q_START; i < Q_START + PART_CAN_DOF; i++) {
        std::cout << "torque_desired_[" << i << "] = " << torque_desired_[i] << std::endl;
    }
    std::cout << "test3-3-6" << std::endl;
    // std::cout << "shm_msgs_->cmd_lower : " << shm_msgs_->cmd_lower << std::endl;
    shm_msgs_->cmd_lower = false;
    std::cout << "test3-3-7" << std::endl;
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