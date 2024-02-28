#ifndef ODRIVE_INTERFACE_HPP_
#define ODRIVE_INTERFACE_HPP_

#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

struct DyrosBoltInitArgs
{
    char port1[20];
    char port2[20];

    int period_ns;
    int lock_core;

    int can_device; // odrv 1, odrv 2
    int can_slave_num;
    int can_slave_start_num;
    int q_start_;

    bool is_main;

    bool verbose;

    char commutation_cache_file[100]; // = "/home/dyros/.tocabi_bootlog/commutationlog";
    char zeropoint_cache_file[100];   // = "/home/dyros/.tocabi_bootlog/zeropointlog";
};

namespace odrive {
    // ODrive command IDs
    enum ODriveCommandId {
        CANOPEN_NMT_MESSAGE = 0x000,
        HEARTBEAT_MESSAGE = 0x001,
        ESTOP_MESSAGE = 0x002,
        GETMOTORERROR = 0x003,
        GETENCODERERROR = 0x004,
        SET_AXIS_NODE_ID = 0x006,
        SET_AXIS_REQUESTED_STATE = 0x007,
        SET_AXIS_STARTUP_CONFIG = 0x008,
        GET_ENCODER_ESTIMATE = 0x009,
        GET_ENCODER_COUNT = 0x00A,
        SET_CONTROLLER_MODES = 0x00B,
        SET_INPUT_POS = 0x00C,
        SET_INPUT_VELOCITY = 0x00D,
        SET_INPUT_TORQUE = 0x00E,
        SET_LIMITS = 0x00F,
        START_ANTI_COGGING = 0x010,
        SET_TRAJ_VEL_LIMIT = 0x011,
        SET_TRAJ_ACCEL_LIMITS = 0x012,
        SET_TRAJ_INERTIA = 0x013,
        GET_IQ = 0x014,
        GET_TEMPERATURE = 0x015,
        REBOOT_ODRIVE = 0x016,
        GET_BUS_VOLTAGE_AND_CURRENT = 0x017,
        CLEAR_ERRORS = 0x018,
        SET_ABSOLUTE_POSITION = 0x019,
        SET_POSITION_GAIN = 0x01A,
        SET_VEL_GAINS = 0x01B,
        CANOPEN_HEARTBEAT_MESSAGE = 0x700
    };

    enum ODriveAxisState {
        UNDEFINED = 0x00,
        IDLE = 0x01,
        STARTUP_SEQUENCE = 0x02,
        FULL_CALIBRATION_SEQUENCE = 0x03,
        MOTOR_CALIBRATION = 0x04,
        ENCODER_INDEX_SEARCH = 0x06,
        ENCODER_OFFSET_CALIBRATION = 0x07,
        CLOSED_LOOP_CONTROL = 0x08,
        LOCKIN_SPIN = 0x09,
        ENCODER_DIR_FIND = 0x0A,
        HOMING = 0x0B,
        ENCODER_HALL_POLARITY_CALIBRATION = 0x0C,
        ENCODER_HALL_PHASE_CALIBRATION = 0x0D
    };

    enum ODriveControlMode {
        VOLTAGE_CONTROL = 0x0,
        TORQUE_CONTROL = 0x1,
        VELOCITY_CONTROL = 0x2,
        POSITION_CONTROL = 0x3
    };

    enum ODriveInputMode {
        INACTIVE = 0x0,
        PASSTHROUGH = 0x1,
        VEL_RAMP = 0x2,
        POS_FILTER = 0x3,
        MIX_CHANNELS = 0x4,
        TRAP_TRAJ = 0x5,
        TORQUE_RAMP = 0x6,
        MIRROR= 0x7,
        TUNING= 0x8
    };

    class ODriveSocketCan {
        public:
            ODriveSocketCan(ros::NodeHandle &nh);
            ~ODriveSocketCan() { disengage();}
            double getAxisAngle();
            double getAxisVelocity();
            double getAxisVoltage();
            double getAxisCurrent();
            void engage();
            void disengage();
            
            void setInputPosition(int axis_can_id_, double position);
            void setInputVelocity(int axis_can_id_, double velocity);
            void setInputTorque(int axis_can_id_, double torque);

            void requestODriveCmd(int axis_can_id_, ODriveCommandId cmd);
            void resetEncoder(int axis_can_id_, ODriveCommandId cmd);
            void setAxisRequestedState(int axis_can_id_, ODriveAxisState state);
            
            std::string axis_name_;
            std::vector<int> axis_can_ids_list;
            
            int axis_error[6] = {0,0,0,0,0,0};
            int axis_current_state[6] = {0,0,0,0,0,0};
            int axis_controller_state[6] = {0,0,0,0,0,0};
            
            double axis_angle[6] = {0,0,0,0,0,0};
            double axis_velocity[6] = {0,0,0,0,0,0};
            double axis_current[6] = {0,0,0,0,0,0};

        private:
            ros::NodeHandle node;
            int socketcan;

            double update_rate_;
            
            void canReceiveMessages();

            void requestBusVoltageAndCurrent(int axis_can_id_);
            

            void setControllerModes(int axis_can_id_, ODriveControlMode control_mode, ODriveInputMode input_mode);

            uint32_t createCanId(int axis_can_id, int command);
            float bytesToFloat(const unsigned char* data);
            int bytesToInt(const unsigned char* data);
    };
}

void *odrvCanThread1(void *data);
void *odrvCanThread2(void *data);
void *odrvCanThread3(void *data);
// void ethercatCheck(TocabiInitArgs *targs);

// double elmoJointMove(double init, double angle, double start_time, double traj_time);

bool controlWordGenerate(const uint16_t statusWord, uint16_t &controlWord);
void checkFault(const uint16_t statusWord, int slave);
// void ecatDiagnoseOnChange();
// void ecatDiagnose();
void cnt_print(int cnt);

bool initDyrosBoltArgs(const DyrosBoltInitArgs &args);
bool initDyrosBoltSystem(const DyrosBoltInitArgs &args);
void cleanupDyrosBoltSystem();

void odrvInit();

void checkJointSafety();
void checkJointStatus();

//void initSharedMemory();
void sendJointStatus();
void getJointCommand();

bool saveCommutationLog();
bool loadCommutationLog(struct timespec &commutation_time);

bool saveZeroPoint();
bool loadZeroPoint(bool force = false);


void emergencyOff();

int kbhit(void);

int getElmoState(uint16_t state_bit);

void findzeroLeg();
void findZeroPointlow(int slv_number, double time_real_);
void findZeroPoint(int slv_number, double time_real_);

void getErrorName(int err_register, char *err);

long getTimeDiff(struct timespec &from, struct timespec &to);
long getTimeDiff(struct timespec &a);

void shutdownSystem();

#endif // ODRIVE_INTERFACE_HPP_
