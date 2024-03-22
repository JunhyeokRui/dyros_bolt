#define CNT_TO_RAD_46 (3.141592 * 2 / (8192 * 100)) //819200
#define CNT_TO_RAD_80 (3.141592 * 2 / (8000 * 100)) //819200

// #define DEG2RAD (3.141592f / 180.0f)

#define EXT_CNT_TO_RAD_46 (3.141592 * 2 / 8192) //819200
#define EXT_CNT_TO_RAD_80 (3.141592 * 2 / 8192) //819200

#define RAD_TO_CNT_46 (1 / (CNT_TO_RAD_46))
#define RAD_TO_CNT_80 (1 / (CNT_TO_RAD_80))

#define EXT_RAD_TO_CNT_46 (1 / (EXT_CNT_TO_RAD_46))
#define EXT_RAD_TO_CNT_80 (1 / (EXT_CNT_TO_RAD_80))

#define EC_TIMEOUTMON 500

#define EC_PACKET_TIMEOUT 250

#define PERIOD_OVF 50

#define ODRV_CNT 3

#define ODRV_DOF 6

// #define ELMO_DOF_UPPER 18

// #define ELMO_DOF_LOWER ELMO_DOF - ELMO_DOF_UPPER

// #define LEG_DOF 6

#define CL_LOCK 50

// #define UPPERBODY_DOF 21

#define PERIOD_NS 500000
#define SEC_IN_NSEC 1000000000UL
#define NSEC_PER_SEC    (1000000000) /* The number of nanoseconds per second. */


#define FORCE_CONTROL_MODE false

extern const char ifname_lower[];
extern char ifname_lower2[];
extern const char ifname_upper[];
extern char ifname_upper2[];

extern const int starting_point;

enum ODRV
{
 ODRV_1,
 ODRV_2,
 ODRV_3

};

namespace MODEL
{

    enum MODEL
    {
        FL_HAA,
        FL_HFE,
        FL_KFE,
        FR_HAA,
        FR_HFE,
        FR_KFE

    };

}

extern const char ODRV_NAME[ODRV_CNT][10];

//pos[i] = pos_elmo[JointMap[i]]
extern const int JointMap[ODRV_DOF];

//pos_elmo[i] = pos[JointMap2[i]]
extern const int JointMap2[ODRV_DOF];

extern const double CNT2RAD[ODRV_DOF];
extern const double EXTCNT2RAD[ODRV_DOF] ;

extern const double RAD2CNT[ODRV_DOF] ;

extern const double EXTRAD2CNT[ODRV_DOF] ;

// extern const double NM2CNT[ODRV_DOF] ;

// extern const int q_ext_mod_odrv_[ODRV_DOF];
// //right -> left
// //right hippitch front -> modval +
// //left knee pitch front -> modval -
// //right ankle pitch up -> modval

// extern const double joint_velocity_limit[ODRV_DOF] ;

// extern const double joint_upper_limit[ODRV_DOF] ;

// extern const double joint_lower_limit[ODRV_DOF];

// extern const double elmo_axis_direction[ODRV_DOF];

// extern const double elmo_ext_axis_direction[ODRV_DOF];

// extern const double upper_homming_minimum_required_length[ODRV_DOF];

// extern const double pos_p_gain[ODRV_DOF];

// extern const double pos_d_gain[ODRV_DOF];
