#ifndef DYROS_BOLT_H
#define DYROS_BOLT_H

#include <iostream>

#define MODEL_DOF 6
#define ODRIVE_CNT 3

namespace DYROS_BOLT {
    const std::string JOINT_NAME[MODEL_DOF] = {
        "FL_HAA", "FL_HFE", "FL_KFE", "FR_HAA", "FR_HFE", "FR_KFE"};
    
    const int JOINT_ID[MODEL_DOF] = {
        1,2,3,
        4,5,6};

    const std::string ODRIVE_NAME[ODRIVE_CNT] = {
        "HAA", "HFE", "KFE"};
    
    enum {
        FL_HAA, 
        FL_HFE, 
        FL_KFE, 
        FR_HAA, 
        FR_HFE, 
        FR_KFE
    }; 
    //Usage : ODIRVE[JointMap[i]] = MODEL[i]
    const int JointMap[MODEL_DOF] = {
        FL_HAA, 
        FL_HFE, 
        FL_KFE, 
        FR_HAA, 
        FR_HFE, 
        FR_KFE
    };
    const double NM2CNT[MODEL_DOF] = {
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    };
    const std::string ACTUATOR_NAME[MODEL_DOF] = {
        "FL_HAA", 
        "FL_HFE", 
        "FL_KFE", 
        "FR_HAA", 
        "FR_HFE", 
        "FR_KFE"
    };
    // static constexpr const char *LINK_NAME[MODEL_DOF + 3] = {
    //     "base_link",
    //     "FL_SHOULDER",
    //     "FL_UPPER_LEG",
    //     "FL_LOWER_LEG",
    //     "FL_FOOT",
    //     "FR_SHOULDER",
    //     "FR_UPPER_LEG",
    //     "FR_LOWER_LEG",
    //     "FR_FOOT"
    // };
    static constexpr const char *LINK_NAME[MODEL_DOF + 1] = {
        "base_link",
        "FL_SHOULDER",
        "FL_UPPER_LEG",
        "FL_LOWER_LEG",
        "FR_SHOULDER",
        "FR_UPPER_LEG",
        "FR_LOWER_LEG"
    };
    const std::string POSITIONACTUATOR_NAME[MODEL_DOF] = {
        "FL_HAA", 
        "FL_HFE", 
        "FL_KFE", 
        "FR_HAA", 
        "FR_HFE", 
        "FR_KFE"
    };
    const int base_link = 0;
    const int FL_LOWER_LEG = 3;
    const int FR_LOWER_LEG = 6;
    // const int FL_FOOT = 4;
    // const int FR_FOOT = 8;
    const int COM_id = 7;


    const int LEFT = 0;
    const int RIGHT = 1;
} //CLASS - DYROS_BOLT

#define _MAXTORQUE 1500

//static atomic
const std::string cred("\033[0;31m");
const std::string creset("\033[0m");
const std::string cblue("\033[0;34m");
const std::string cgreen("\033[0;32m");
const std::string cyellow("\033[0;33m");

#endif