#pragma once

// #include <atomic>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ipc.h>
#include <sys/shm.h>
// #include <cstring>
#include <sys/stat.h>
#include <math.h>
#include <vector>
#include <vector>

#include "dyros_bolt_real_robot/odrive_socketcan_settings.h"
#include "shm_msgs.h"
#include "dyros_bolt_real_robot/odrive_socketcan.h"
// odrive::ODriveSocketCan odrv;    

// Function to calculate squared norm of a float array


struct DyrosBoltInitArgs
{
    char port1[20];
    char port2[20];

    int period_ns;
    int lock_core;

    int can_device; // ELMO 1, ELMO 2
    int can_slave_num;
    int q_start_;

    bool is_main;

    bool verbose;

    char commutation_cache_file[100]; // = "/home/dyros/.tocabi_bootlog/commutationlog";
    char zeropoint_cache_file[100];   // = "/home/dyros/.tocabi_bootlog/zeropointlog";
};


// float k_tau[MODEL_DOF] = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
// float torque_desired_[MODEL_DOF];
// float extencoder_offset_[MODEL_DOF];
// float imu_data_quat[4]; 
// float imu_accelometer_[3]; // current accelometer values
// float imu_angular_velocity[3];
// float q_[MODEL_DOF];      // sendstate
// float q_dot_[MODEL_DOF];  // sendstate
// float torque_[MODEL_DOF]; // sendstate
// float q_ext_[MODEL_DOF];
// float q_ext_offset_[MODEL_DOF] ;

// bool extencoder_init_flag_;
// unsigned int joint_id_[MODEL_DOF];
// unsigned int joint_id_inversed_[MODEL_DOF];
// unsigned int control_mask_[MODEL_DOF];
void odrvInit();
void makeIDInverseList();
void readDevice();
void update();
void writeDevice();

double elmoJointMove(double init, double angle, double start_time, double traj_time);

bool controlWordGenerate(const uint16_t statusWord, uint16_t &controlWord);
void checkFault(const uint16_t statusWord, int slave);
void ecatDiagnoseOnChange();
void ecatDiagnose();
void cnt_print(int cnt);

bool initDyrosBoltArgs(const DyrosBoltInitArgs &args);
bool initDyrosBoltSystem(const DyrosBoltInitArgs &args);
void cleanupDyrosBoltSystem();


void checkJointSafety();
void checkJointStatus();

//void initSharedMemory();
void sendJointStatus();
void getJointCommand();
void *ethercatThread1(void *data);
bool saveCommutationLog();
bool loadCommutationLog(struct timespec &commutation_time);

bool saveZeroPoint();


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
float squaredNorm(const float* arr, size_t size);
bool areMotorsReady();